# provide data for SplaTam
import os
import numpy as np
import shutil
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--source_path", "-s", required=True, type=str, help="Input data")
args = parser.parse_args()
from convert_pose8_to_pose12 import convert_pose8_to_pose12


def sort_by_number(filename):
    return int(''.join(filter(str.isdigit, filename)))


def generate_splatam(args):
    sp_path = os.path.join(args.source_path, "splatam")
    os.makedirs(sp_path, exist_ok=True)
    os.makedirs(os.path.join(sp_path, "rgb"), exist_ok=True)
    os.makedirs(os.path.join(sp_path, "depth"), exist_ok=True)
    rgb = os.listdir(os.path.join(args.source_path, "images_lidar_loop"))
    depth = os.listdir(os.path.join(args.source_path, "depths_lidar_loop"))
    rgb = sorted(rgb, key=sort_by_number)
    depth = sorted(depth, key=sort_by_number)
    for i, img in enumerate(rgb):
        img_path = os.path.join(args.source_path, "images_lidar_loop", img)
        depth_path = os.path.join(args.source_path, "depths_lidar_loop", depth[i])
        shutil.copy(img_path, os.path.join(sp_path, "rgb", img))
        shutil.copy(depth_path, os.path.join(sp_path, "depth", depth[i]))
    with open(os.path.join(sp_path, "rgb.txt"), "w") as f:
        for i in range(len(rgb)):
            f.write(f"{i} rgb/{rgb[i]}\n")
    with open(os.path.join(sp_path, "depth.txt"), "w") as f:
        for i in range(len(rgb)):
            f.write(f"{i} depth/{depth[i]}\n")
    with open(os.path.join(sp_path, "groundtruth.txt"), "w") as f:
        text = os.path.join(args.source_path, "groundtruth_loop.txt")
        with open(text, "r") as f1:
            lines = f1.readlines()
        for i, line in enumerate(lines):
            line = line.strip()
            line = line.split(" ")
            line[0] = i
            f.write(f"{line[0]} {line[1]} {line[2]} {line[3]} {line[4]} {line[5]} {line[6]} {line[7]}\n")


def generate_nerfpp(args):
    nerf_path = os.path.join(args.source_path, "nerfplusplus")
    os.makedirs(nerf_path, exist_ok=True)
    train = os.path.join(nerf_path, "train")
    os.makedirs(os.path.join(train, "rgb"), exist_ok=True)
    os.makedirs(os.path.join(train, "intrinsics"), exist_ok=True)
    os.makedirs(os.path.join(train, "pose"), exist_ok=True)
    test = os.path.join(nerf_path, "test")
    # os.makedirs(os.path.join(test, "rgb"), exist_ok=True)
    # os.makedirs(os.path.join(test, "intrinsics"), exist_ok=True)
    # os.makedirs(os.path.join(test, "pose"), exist_ok=True)
    validation = os.path.join(nerf_path, "validation")
    os.makedirs(os.path.join(validation, "rgb"), exist_ok=True)
    os.makedirs(os.path.join(validation, "intrinsics"), exist_ok=True)
    os.makedirs(os.path.join(validation, "pose"), exist_ok=True)
    rgb = os.listdir(os.path.join(args.source_path, "images_lidar"))
    depth = os.listdir(os.path.join(args.source_path, "depths_lidar"))
    rgb = sorted(rgb, key=sort_by_number)
    depth = sorted(depth, key=sort_by_number)
    for i in range(len(rgb)):
        if i % 4 == 0:
            inputname = os.path.join(args.source_path, "images_lidar", rgb[i])
            filename = os.path.join(train, "rgb", rgb[int(i / 2)])
            shutil.copy(inputname, filename)
            with open(os.path.join(train, "intrinsics", "intrinsics_{}.txt".format(int(i / 2))), "w") as f1:
                f1.write(
                    str(332.232689) + " " + str(0.) + " " + str(333.058485) + " " + str(0.) + " " + str(0.) + " " + str(
                        332.644823) + " " + str(240.998586) + " " + str(0.) + " " + str(0.) + " " + str(0.) + " " + str(
                        1.) + " " + str(0.) + " " + str(0.) + " " + str(0.) + " " + str(0.) + " " + str(1.) + "\n")
        if i % 4 == 2:
            inputname = os.path.join(args.source_path, "images_lidar", rgb[i])
            filename = os.path.join(validation, "rgb", rgb[int(i / 2)])
            shutil.copy(inputname, filename)
            with open(os.path.join(validation, "intrinsics", "intrinsics_{}.txt".format(int(i / 2))), "w") as f1:
                f1.write(
                    str(332.232689) + " " + str(0.) + " " + str(333.058485) + " " + str(0.) + " " + str(0.) + " " + str(
                        332.644823) + " " + str(240.998586) + " " + str(0.) + " " + str(0.) + " " + str(0.) + " " + str(
                        1.) + " " + str(0.) + " " + str(0.) + " " + str(0.) + " " + str(0.) + " " + str(1.) + "\n")

    H_list, max_ = convert_pose8_to_pose12(os.path.join(args.source_path, "test.txt"))
    for i, H in enumerate(H_list):
        R_calib = np.array([
            [-0.01783068, -0.99978912, 0.01020197],
            [-0.02988329, -0.00966631, -0.99950683],
            [0.99939454, -0.01812658, -0.02970480]
        ])
        # T_calib = np.array([-0.08032920, 0.10752965, -0.06024161])

        # 构建标定矩阵
        T_calib_homogeneous = np.eye(4)
        T_calib_homogeneous[:3, :3] = R_calib
        # T_calib_homogeneous[:3, 3] = T_calib

        # 计算标定矩阵的逆矩阵
        T_calib_homogeneous_inv = np.linalg.inv(T_calib_homogeneous)

        # 构建激光雷达坐标系下的变换矩阵
        # T_lidar = np.eye(4)
        # T_lidar[:3, :3] = rot_matrix
        # T_lidar[:3, 3] = translation

        T_camera = T_calib_homogeneous @ H @ T_calib_homogeneous_inv
        H_list[i] = np.linalg.inv(T_camera)
    H_array = np.array(H_list)
    print(H_array.shape)
    x_max, x_min = np.max(H_array[:, 0, 3]), np.min(H_array[:, 0, 3])
    y_max, y_min = np.max(H_array[:, 1, 3]), np.min(H_array[:, 1, 3])
    z_max, z_min = np.max(H_array[:, 2, 3]), np.min(H_array[:, 2, 3])
    center = np.array([(x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2])
    radius = np.array([(x_max - x_min) / 2, (y_max - y_min) / 2, (z_max - z_min) / 2])
    radius = np.linalg.norm(radius)

    for i, H in enumerate(H_list):
        if i % 4 == 0:
            with open(os.path.join(train, "pose", "pose_{}.txt".format(int(i / 2))), "w") as f2:
                H = H.tolist()
                f2.write(str(H[0][0]) + " " + str(H[0][1]) + " " + str(H[0][2]) + " " + str(
                    (H[0][3] - center[0]) / radius) + " " + str(
                    H[1][0]) + " " + str(
                    H[1][1]) + " " + str(H[1][2]) + " " + str((H[1][3] - center[1]) / radius) + " " +
                         str(H[2][0]) + " " + str(H[2][1]) + " " + str(H[2][2]) + " " + str(
                    (H[2][3] - center[2]) / radius) + " " +
                         str(H[3][0]) + " " + str(H[3][1]) + " " + str(H[3][2]) + " " + str(H[3][3]) +
                         "\n")
        if i % 4 == 2:
            with open(os.path.join(validation, "pose", "pose_{}.txt".format(int(i / 2))), "w") as f2:
                H = H.tolist()
                f2.write(str(H[0][0]) + " " + str(H[0][1]) + " " + str(H[0][2]) + " " + str(
                    (H[0][3] - center[0]) / radius) + " " + str(
                    H[1][0]) + " " + str(
                    H[1][1]) + " " + str(H[1][2]) + " " + str((H[1][3] - center[1]) / radius) + " " +
                         str(H[2][0]) + " " + str(H[2][1]) + " " + str(H[2][2]) + " " + str(
                    (H[2][3] - center[2]) / radius) + " " +
                         str(H[3][0]) + " " + str(H[3][1]) + " " + str(H[3][2]) + " " + str(H[3][3]) +
                         "\n")
    shutil.copytree(train, test)


def generate_3dgs(args):
    gaussian_path = os.path.join(args.source_path, "gaussian")
    os.makedirs(gaussian_path, exist_ok=True)
    images_path = os.path.join(gaussian_path, "images")
    os.makedirs(images_path, exist_ok=True)
    rgb = os.listdir(os.path.join(args.source_path, "images_lidar_loop"))
    rgb = sorted(rgb, key=sort_by_number)
    for i, image in enumerate(rgb):
        if i % 2 == 0:
            inputname = os.path.join(args.source_path, "images_lidar_loop", image)
            filename = os.path.join(images_path, rgb[int(i / 2)])
            shutil.copy(inputname, filename)


if __name__ == "__main__":
    generate_splatam(args)
    # generate_nerfpp(args)
    generate_3dgs(args)