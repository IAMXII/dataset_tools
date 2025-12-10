import os
import json
import cv2
import numpy as np
from pycocotools import mask as mask_utils
from tifffile import astype

json_root = "/home/liuwei/mnt/instant_vggt_dataset/moving_dataset/MOVE/MOVE_release/annotations"      # 你的 JSON 文件夹路径
output_root = "/home/liuwei/mnt/instant_vggt_dataset/moving_dataset/MOVE/MOVE_release/masks"  # 保存 mask 的根目录
os.makedirs(output_root, exist_ok=True)

# 遍历所有 JSON 文件
for json_name in os.listdir(json_root):
    if not json_name.endswith(".json"):
        continue

    json_path = os.path.join(json_root, json_name)

    # 解析 JSON 文件名作为文件夹名
    seq_name = os.path.splitext(json_name)[0]
    save_dir = os.path.join(output_root, seq_name)
    os.makedirs(save_dir, exist_ok=True)

    print(f"Processing {json_name} → {save_dir}")

    # 加载 JSON
    with open(json_path) as f:
        data = json.load(f)
    data_m = data["objects"][0]["masks"]



    # 遍历 JSON 内所有帧
    for frame_id,ann in enumerate(data_m):
        try:
            h,w = ann["size"]
            rle = ann
            rle["counts"] = rle["counts"].encode()

            mask = mask_utils.decode(rle)  # (H, W), 0/1
        except:
            # print(json_name)
            mask = (np.ones((h,w))*255).astype(np.uint8)
        # frame_id = ann["frame_id"]



        # 保存为 PNG
        mask_png = (mask * 255).astype(np.uint8)
        save_path = os.path.join(save_dir, f"{frame_id:05d}.png")

        cv2.imwrite(save_path, mask_png)

    print(f"→ Done: {json_name}\n")
