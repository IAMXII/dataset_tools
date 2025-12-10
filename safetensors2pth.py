#!/usr/bin/env python3
"""
Batch segmentation + binary mask exporter

Example:
    python batch_seg_inundation.py \
        --input_dir /path/to/images \
        --output_dir ./out_masks \
        --checkpoint smp-hub/segformer-b5-1024x1024-city-160k \
        --target-classes 11,12

If you want to treat class indices 11 and 12 as "inundation" (flooded),
they will be combined to a single binary mask (logical OR).
"""

import os
import glob
import argparse
from pathlib import Path
from tqdm import tqdm

import numpy as np
from PIL import Image

import torch
import albumentations as A
import segmentation_models_pytorch as smp


def load_image_pil(path):
    img = Image.open(path).convert("RGB")
    arr = np.array(img)
    return img, arr


def ensure_dir(d):
    os.makedirs(d, exist_ok=True)


def save_seg_as_png(seg_arr, out_path):
    """
    seg_arr: 2D numpy array of dtype int (H,W) with class indices
    Save as grayscale PNG (values are class indices; use palette externally if needed)
    """
    seg_img = Image.fromarray(seg_arr.astype(np.uint8))
    seg_img.save(out_path, compress_level=1)


def save_binary_mask(mask_bool, out_path):
    """
    mask_bool: 2D boolean numpy array
    """
    mask_u8 = (mask_bool.astype(np.uint8) * 255)
    Image.fromarray(mask_u8).save(out_path, compress_level=1)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", required=True, help="Directory with input images")
    parser.add_argument("--output_dir", required=True, help="Directory to save outputs")
    parser.add_argument("--checkpoint", default="smp-hub/segformer-b5-1024x1024-city-160k",
                        help="smp.from_pretrained checkpoint id or local path")
    parser.add_argument("--device", default=None, help="cuda or cpu (default auto)")
    parser.add_argument("--batch_size", type=int, default=1, help="Batch size for inference")
    parser.add_argument("--ext", default="jpg,png,jpeg", help="Image extensions to include (comma separated)")
    parser.add_argument("--target-classes", dest="target_classes", default=None,
                        help="Comma-separated list of class indices to consider as inundation (e.g. 11,12). If omitted, only full seg saved")
    parser.add_argument("--save-npy", action="store_true", help="Also save numpy .npy mask files")
    parser.add_argument("--resize", type=int, default=None,
                        help="Optional: resize short edge to this before preprocessing (keeps aspect) - rarely needed")
    return parser.parse_args()


def main():
    args = parse_args()
    device = args.device if args.device else ("cuda" if torch.cuda.is_available() else "cpu")

    exts = [e.strip().lower() for e in args.ext.split(",")]

    # gather images
    image_paths = []
    for e in exts:
        image_paths += sorted(glob.glob(os.path.join(args.input_dir, f"**/*.{e}"), recursive=True))
    if len(image_paths) == 0:
        print("No images found in", args.input_dir)
        return

    ensure_dir(args.output_dir)
    seg_out_dir = os.path.join(args.output_dir, "seg")
    mask_out_dir = os.path.join(args.output_dir, "mask")
    ensure_dir(seg_out_dir)
    ensure_dir(mask_out_dir)

    # parse target classes
    if args.target_classes:
        target_classes = [int(x) for x in args.target_classes.split(",") if x.strip() != ""]
    else:
        target_classes = []

    # -----------------------------
    # load model and preprocessing
    # -----------------------------
    print("Loading model:", args.checkpoint)
    model = smp.from_pretrained(args.checkpoint).to(device).eval()

    print("Loading preprocessing for checkpoint (albumentations)...")
    # Note: A.Compose.from_pretrained returns an albumentations transform which may vary by model
    preprocessing = A.Compose.from_pretrained(args.checkpoint)

    # -----------------------------
    # Helper to run one image through model
    # -----------------------------
    def infer_single(np_img):
        # np_img: H,W,3 uint8
        # albumentations expects dict with key "image"
        aug = preprocessing(image=np_img)
        x = aug["image"]  # likely HWC with float and normalized
        # ensure C,H,W and float32
        x_t = torch.as_tensor(x).permute(2, 0, 1).unsqueeze(0).to(device)  # 1,C,H,W
        with torch.no_grad():
            out = model(x_t)  # shape (1, num_classes, h_out, w_out)
        return out

    # -----------------------------
    # batch processing loop
    # -----------------------------
    for p in tqdm(image_paths, desc="images"):
        try:
            pil_img, np_img = load_image_pil(p)
        except Exception as e:
            print("Failed to open", p, "->", e)
            continue

        # optional resize before preprocessing (keeps aspect)
        if args.resize is not None:
            # resize such that short edge == args.resize
            H0, W0 = np_img.shape[0], np_img.shape[1]
            short = min(H0, W0)
            scale = args.resize / short
            new_h = int(round(H0 * scale))
            new_w = int(round(W0 * scale))
            pil_img = pil_img.resize((new_w, new_h), Image.BICUBIC)
            np_img = np.array(pil_img)

        out = infer_single(np_img)  # torch tensor on device

        # upsample to original image size (H_orig, W_orig)
        H_orig, W_orig = pil_img.height, pil_img.width
        out_up = torch.nn.functional.interpolate(out, size=(H_orig, W_orig),
                                                 mode="bilinear", align_corners=False)
        # argmax to class index map
        seg = out_up.argmax(1).squeeze(0).cpu().numpy().astype(np.uint8)  # (H,W)

        base = Path(p).stem
        seg_path = os.path.join(seg_out_dir, f"{base}_seg.png")
        save_seg_as_png(seg, seg_path)

        if len(target_classes) > 0:
            # build binary mask: True where seg in target_classes
            mask_bool = np.isin(seg, target_classes)
            mask_path = os.path.join(mask_out_dir, f"{base}.png")
            save_binary_mask(mask_bool, mask_path)
            if args.save_npy:
                np.save(os.path.join(mask_out_dir, f"{base}_mask.npy"), mask_bool.astype(np.uint8))
        else:
            # no target classes specified: optionally still save a "foreground" mask (non-zero)
            pass

    print("Done. Seg saved to:", seg_out_dir)
    if len(target_classes) > 0:
        print("Masks saved to:", mask_out_dir)


if __name__ == "__main__":
    from pathlib import Path
    main()
