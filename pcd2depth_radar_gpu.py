import cv2
import numpy as np
import torch
import open3d as o3d
import os
import yaml
from tqdm import tqdm
from time import time


def sort_by_number(filename):
    return int(''.join(filter(str.isdigit, filename)))


# 读取相机标定 yaml 文件
with open("ntu.yaml", "r") as f:   # 修改为你的 yaml 文件路径
    config = yaml.safe_load(f)

# 提取外参 (lidar_to_cam)
P = torch.tensor(config["lidar_to_cam"]["extrinsics"], dtype=torch.float32, device="cuda")  # (4,4)
# 提取内参
K = torch.tensor(config["intrinsics"], dtype=torch.float32, device="cuda")  # (3,3)
# 图像大小
img_w, img_h = config["image_size"]

# 点云路径
pointcloud_path = '/home/liuwei/zyf/cp/pcds_radar'
pointcloud = sorted(os.listdir(pointcloud_path), key=sort_by_number)

save_path = "/home/liuwei/zyf/cp/depths_radar"
os.makedirs(save_path, exist_ok=True)

if __name__ == "__main__":
    for i, pcd_file in tqdm(enumerate(pointcloud)):
        start_time = time()

        # 读取点云
        pcd = o3d.io.read_point_cloud(os.path.join(pointcloud_path, pcd_file))
        points = np.asarray(pcd.points, dtype=np.float32)

        if points.shape[0] == 0:
            continue

        # 转到 GPU
        points_gpu = torch.tensor(points, dtype=torch.float32, device="cuda")

        # 齐次坐标 (N,4)
        ones = torch.ones((points_gpu.shape[0], 1), dtype=torch.float32, device="cuda")
        points_h = torch.cat((points_gpu, ones), dim=1)  # (N,4)

        # 雷达到相机坐标 (N,4) -> (N,3)
        points_cam = (P @ points_h.T).T[:, :3]  # (N,3)

        # 过滤相机后方的点
        mask = points_cam[:, 2] > 0
        points_cam = points_cam[mask]
        if points_cam.shape[0] == 0:
            continue

        # 投影到像素平面 (N,3)
        proj = (K @ points_cam.T).T  # (N,3)
        u = torch.floor(proj[:, 0] / proj[:, 2]).to(torch.int32)
        v = torch.floor(proj[:, 1] / proj[:, 2]).to(torch.int32)
        depth = (points_cam[:, 2] * 400).to(torch.int32)  # 深度值缩放

        # 过滤图像范围
        valid_mask = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
        u = u[valid_mask]
        v = v[valid_mask]
        depth = depth[valid_mask]

        if u.shape[0] == 0:
            continue

        # 初始化深度图为大值
        depth_image_gpu = torch.full((img_h, img_w), 2**31-1, dtype=torch.int32, device="cuda")

        # 展平索引
        flat_idx = v * img_w + u

        # 对同一像素取最小深度
        depth_image_gpu = depth_image_gpu.view(-1)
        depth_image_gpu.scatter_reduce_(0, flat_idx, depth, reduce="amin", include_self=True)
        depth_image_gpu = depth_image_gpu.view(img_h, img_w)

        # 未更新的地方设为 0
        depth_image_gpu[depth_image_gpu == 2**31-1] = 0

        # 转回 CPU 并保存
        depth_image = depth_image_gpu.cpu().numpy().astype(np.uint16)
        cv2.imwrite(f"{save_path}/depth_{i}.png", depth_image)

        end_time = time()
        # print(f"time_{i}: {end_time - start_time:.4f}")
