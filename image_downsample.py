import os
import cv2
import numpy as np
from tqdm import tqdm

def downsample_rgb(image):
    """对RGB图像进行降采样，取区域平均值"""
    h, w, c = image.shape
    # 偶数裁剪，保证可以整除
    h_new, w_new = h // 2, w // 2
    image = image[:h_new * 2, :w_new * 2, :]

    # 按2×2分块取平均
    image_reshaped = image.reshape(h_new, 2, w_new, 2, c)
    image_down = image_reshaped.mean(axis=(1, 3)).astype(np.uint8)
    return image_down

def process_folder(input_folder, output_folder):
    os.makedirs(output_folder, exist_ok=True)
    files = sorted(os.listdir(input_folder))
    for fname in tqdm(files, desc="Processing RGB images"):
        if not fname.lower().endswith((".png", ".jpg", ".jpeg")):
            continue
        fpath = os.path.join(input_folder, fname)
        image = cv2.imread(fpath, cv2.IMREAD_COLOR)  # 读取RGB
        if image is None:
            continue
        image_down = downsample_rgb(image)
        cv2.imwrite(os.path.join(output_folder, fname), image_down)

if __name__ == "__main__":
    input_folder = "/home/liuwei/mnt/traj_data/hitsz_loop/images_radar"      # 输入文件夹
    output_folder = "/home/liuwei/mnt/traj_data/hitsz_loop/images_radar_d2" # 输出文件夹
    process_folder(input_folder, output_folder)
