import os
import cv2
import shutil
import numpy as np

def sort_by_number(filename):
    """假设文件名里有数字，用于排序"""
    return int(''.join(filter(str.isdigit, filename)))

def extract_frames(img_dir, save_dir, num_frames=100):
    os.makedirs(save_dir, exist_ok=True)

    # 获取所有图片
    images = sorted(os.listdir(img_dir), key=sort_by_number)
    total = len(images)

    if total < num_frames:
        raise ValueError(f"数据集只有 {total} 张图，不足 {num_frames} 张")

    # 等间隔索引
    indices = np.linspace(0, total - 1, num_frames, dtype=int)

    for i, idx in enumerate(indices):
        src = os.path.join(img_dir, images[idx])
        dst = os.path.join(save_dir, f"{i:04d}.png")  # 统一命名
        shutil.copy(src, dst)

    print(f"✅ 已保存 {num_frames} 张图片到 {save_dir}")

# 使用示例
if __name__ == "__main__":
    img_dir = "/home/liuwei/mnt/vggt_dataset/cp/images_lidar_loop/undistorted_images"       # 原始数据集路径
    save_dir = "/home/liuwei/mnt/vggt_dataset/cp/images_corner/images_1000"  # 保存路径
    extract_frames(img_dir, save_dir, num_frames=1000)
