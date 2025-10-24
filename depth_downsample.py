import os
import cv2
import numpy as np
from tqdm import tqdm

def downsample_max(img):
    """
    对 16 位深度图做 2×2 降采样，取最大值
    输入: img (H, W), dtype=np.uint16
    输出: (H//2, W//2), dtype=np.uint16
    """
    H, W = img.shape
    # 取 2x2 block 最大值
    img = img[:H//2*2, :W//2*2]  # 保证偶数尺寸
    img_reshape = img.reshape(H//2, 2, W//2, 2)
    img_down = img_reshape.max(axis=(1, 3))
    return img_down.astype(np.uint16)

def process_folder(in_dir, out_dir):
    os.makedirs(out_dir, exist_ok=True)

    files = sorted([f for f in os.listdir(in_dir) if f.lower().endswith(".png")])

    for f in tqdm(files, desc="Downsampling"):
        in_path = os.path.join(in_dir, f)
        out_path = os.path.join(out_dir, f)

        # 读入16位深度图
        img = cv2.imread(in_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            print(f"跳过无法读取的文件: {in_path}")
            continue
        if img.dtype != np.uint16:
            print(f"警告: {f} 不是16位图像，dtype={img.dtype}")

        # 降采样
        img_down = downsample_max(img)

        # 保存
        cv2.imwrite(out_path, img_down)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("in_dir", help="输入文件夹 (包含16位深度图)")
    parser.add_argument("out_dir", help="输出文件夹 (保存降采样后的图)")
    args = parser.parse_args()

    process_folder(args.in_dir, args.out_dir)
    print("完成 ✅")
