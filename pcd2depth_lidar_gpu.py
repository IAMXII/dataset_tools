import cv2
import numpy as np
import torch
import open3d as o3d
import os
import yaml
from tqdm import tqdm
from time import time
import random

def sort_by_number(filename):
    return int(''.join(filter(str.isdigit, filename)))


# 读取相机标定 yaml 文件
with open("ntu.yaml", "r") as f:   # 假设文件名叫 calib.yaml
    config = yaml.safe_load(f)

# 提取外参 (lidar_to_cam)
P = torch.tensor(config["lidar_to_cam"]["extrinsics"], dtype=torch.float32, device="cuda")  # (4,4)
# 提取内参
K = torch.tensor(config["intrinsics"], dtype=torch.float32, device="cuda")  # (3,3)
# 图像大小
img_w, img_h = config["image_size"]
# 畸变系数（暂未使用）
dist_coef = torch.tensor(config["coef"], dtype=torch.float32, device="cuda")

# 点云路径
pointcloud_path = '/home/liuwei/zyf/cp/pcds_lidar'
pointcloud = sorted(os.listdir(pointcloud_path), key=sort_by_number)


if __name__ == "__main__":
    for i, pcd_file in tqdm(enumerate(pointcloud)):
        start_time = time()
        pcd = o3d.io.read_point_cloud(os.path.join(pointcloud_path, pcd_file))
        points = np.asarray(pcd.points, dtype=np.float32)
        points[:,2] += np.random.uniform(-0.2, 0.2, points.shape[0])
        points = points[random.sample(range(points.shape[0]), int(points.shape[0] * 0.33))]

        if points.shape[0] == 0:
            continue

        # 转到 GPU
        points_gpu = torch.tensor(points, dtype=torch.float32, device="cuda")

        # 扩展为齐次坐标 (N,4)
        ones = torch.ones((points_gpu.shape[0], 1), dtype=torch.float32, device="cuda")
        points_h = torch.cat((points_gpu, ones), dim=1)  # (N,4)

        # 转换到相机坐标系 (N,4) -> (N,3)
        points_cam = (P @ points_h.T).T[:, :3]  # (N,3)

        # # 过滤 Z<=0 的点
        # 过滤 Z<=0 的点 (必须要加)
        mask = points_cam[:, 2] > 0
        points_cam = points_cam[mask]

        if points_cam.shape[0] == 0:
            continue

        # 投影到像素平面 (N,3)
        proj = (K @ points_cam.T).T  # (N,3)
        u = torch.floor(proj[:, 0] / proj[:, 2]).to(torch.int32)
        v = torch.floor(proj[:, 1] / proj[:, 2]).to(torch.int32)
        depth = (points_cam[:, 2] * 400).to(torch.int32)

        # 过滤图像范围内的点
        valid_mask = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
        u = u[valid_mask]
        v = v[valid_mask]
        depth = depth[valid_mask]

        # 构建深度图，初始化为一个很大的值，而不是 0
        depth_image_gpu = torch.full((img_h, img_w), 2**31-1, dtype=torch.int32, device="cuda")

        # 展平索引
        flat_idx = v * img_w + u

        # 取最小深度
        depth_image_gpu = depth_image_gpu.view(-1)
        depth_image_gpu.scatter_reduce_(0, flat_idx, depth, reduce="amin", include_self=True)
        depth_image_gpu = depth_image_gpu.view(img_h, img_w)

        # 把未赋值的地方（仍是初始值）变成 0
        depth_image_gpu[depth_image_gpu == 2**31-1] = 0

        # 转回 CPU 并保存
        depth_image = depth_image_gpu.cpu().numpy().astype(np.uint16)
        cv2.imwrite(f"/home/liuwei/zyf/smoke/depths_cmdf/depth_{i}.png", depth_image)


        end_time = time()
        # print(f"time_{i}: {end_time - start_time:.4f}")
