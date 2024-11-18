import os
import sys
from PIL import Image
from typing import NamedTuple
from scene.colmap_loader import read_extrinsics_text, read_intrinsics_text, qvec2rotmat, \
    read_extrinsics_binary, read_intrinsics_binary, read_points3D_binary, read_points3D_text
from scene.dataset_readers import readColmapCameras
from utils.graphics_utils import getWorld2View2, focal2fov, fov2focal
import numpy as np
import json
from pathlib import Path
from plyfile import PlyData, PlyElement
from utils.sh_utils import SH2RGB
from scene.gaussian_model import BasicPointCloud

import open3d as o3d
if __name__ == "__main__":
    file_path = "../Downloads/360_v2/ntu_garden"
    ply_path = os.path.join(file_path,"sparse/0/points3D.bin")
    extrinsics_path = os.path.join(file_path, "sparse/0/images.bin")
    intrinsics_path = os.path.join(file_path, "sparse/0/cameras.bin")
    cam_extrinsics = read_extrinsics_binary(extrinsics_path)
    cam_intrinsics = read_intrinsics_binary(intrinsics_path)
    images_folder = os.path.join(file_path, "images")
    xyz, rgb, _ = read_points3D_binary(ply_path)
    cam = readColmapCameras(cam_extrinsics, cam_intrinsics, images_folder)
    R = cam[0].R
    R_inv = np.linalg.inv(R)
    T = cam[0].T
    print(R)
    print(T)
    # ply_data = fetchPly(ply_path)
    # position = ply_data.points
    # print("xyz:",xyz)
    new_position = np.dot(R, (xyz.T))+T.reshape([3,1])
    new_position = np.concatenate((new_position[2].reshape([-1,1]), -new_position[0].reshape([-1,1]), -new_position[1].reshape([-1,1])),axis=1)
    print(new_position.shape)
    # ply_data.points = new_position
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(new_position)

    # 保存为 PCD 文件
    save_path = os.path.join(file_path, "output_with_data.pcd")
    o3d.io.write_point_cloud(save_path, pcd)
print("Done")