import os
import cv2
import numpy as np
import yaml
# import torch
with open("ntu.yaml", "r") as f:   # 假设文件名叫 calib.yaml
    config = yaml.safe_load(f)
# 提取内参
K = np.array(config["intrinsics"])  # (3,3)
# 图像大小
img_w, img_h = config["image_size"]
# 畸变系数（暂未使用）
dist_coef = np.array(config["coef"])
# 相机内参和畸变参数
# camera_matrix = np.array([
#     [332.232689, 0.0, 333.058485],
#     [0.0, 332.644823, 240.998586],
#     [0.0, 0.0, 1.0]
# ])
# dist_coeff = np.array([0.011770, -0.041851, 0.000701, -0.000319, 0.0])

# 输入输出路径
image_folder = "/home/liuwei/mnt/vggt_dataset/cp/images_lidar_loop"
output_folder = os.path.join(image_folder, "undistorted_images")
os.makedirs(output_folder, exist_ok=True)

# 按文件名中的数字排序（如果文件名不是纯数字可以改 key）
def sort_by_number(filename):
    return int(''.join(filter(str.isdigit, filename)) or -1)

image_files = sorted(
    [f for f in os.listdir(image_folder) if f.endswith(('.jpg', '.png', '.jpeg'))],
    key=sort_by_number
)

# 遍历去畸变
for i, img_file in enumerate(image_files):
    img_path = os.path.join(image_folder, img_file)
    img = cv2.imread(img_path)

    if img is None:
        print(f"⚠️ 读取失败: {img_path}")
        continue

    undistorted_img = cv2.undistort(img, K, dist_coef)

    # 6 位整数命名
    undistorted_path = os.path.join(output_folder, f"{i:06d}.png")
    cv2.imwrite(undistorted_path, undistorted_img)

print("✅ 去畸变完成")
