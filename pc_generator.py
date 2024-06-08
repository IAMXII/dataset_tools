import numpy as np
import rosbag
import open3d as o3d
# from sensor_msgs.msg import PointCloud
# from sensor_msgs import point_cloud
import os
from geometry_msgs.msg import Point32
from argparse import ArgumentParser
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R
import glob

ERROR = 1e-3

R_pc = np.array([[-0.0264814146660713, -0.999649386885150, 0.00173906523035761],
                 [0.0121395165905959, -0.00206233611926795, -0.999925722855674],
                 [0.999576628994339, -0.0264570578759927, 0.0121885681396033]])
T_pc = np.array([[0.0659530032036776, 0.108374858163385, 0.0699178702997177]]).T


def read_txt(file_path):
    with open(file_path, 'r') as file:
        data_0 = file.readlines()
    for item in data_0:
        item = item.split()
        item = [eval(it) for it in item]
        data.append(item)
    return data


def pcd2xyz(file):
    pointcloud = o3d.io.read_point_cloud(file)
    points = np.asarray(pointcloud.points)
    return points


# 示例：定义四元数
# quaternion = np.array([0.707, 0.0, 0.707, 0.0])  # 示例四元数，注意顺序可能与其他库有差异
def read_pcd(bag_file):
    bag = rosbag.Bag(bag_file, 'r')
    pc_data = []
    time = []
    for topic, msg, t in tqdm(bag.read_messages(topics=['/radar_enhanced_pcl'])):
        time.append(msg.header.stamp.to_sec())

        if topic == '/radar_enhanced_pcl':
            # points = o3d.io.read_point_cloud(msg)
            points = msg.points
            pc_data.append(points)
    # with open("output.txt",'w') as file:
    #     for item in time:
    #         file.write(str(item)+'\n')
    bag.close()
    print("over")
    data = []
    for points in pc_data:
        x = []
        y = []
        z = []
        for point in points:
            x.append(point.x)
            y.append(point.y)
            z.append(point.z)
        points = np.array([x, y, z])
        points = points.T
        data.append(points)
    return data, time


def generate_pcd(points, pose):
    points = points.T
    points_cur = np.dot(pose.R, points) + pose.T.reshape([3, 1])
    return points_cur.T


def downsample_pcd(pcd, voxel_size):
    # 读取点云
    # 进行降采样
    downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    # 将降采样后的点云转换为NumPy数组
    point_cloud_array = np.asarray(downsampled_pcd.points)
    return point_cloud_array


def project_point_cloud_to_image(cpoints, camera_matrix, Pose, image_size):
    points = []
    ids = []
    x = 0
    # Project each 3D point onto the image plane
    for point in cpoints:
        point_3d = np.array([[point[0], point[1], point[2]]]).T
        R_pc_inv = np.linalg.inv(R_pc)
        point_3d = np.dot(R_pc_inv, point_3d - T_pc)

        # Apply rotation and translation
        transformed_point = np.dot(Pose.R, point_3d) + Pose.T.reshape(3,-1)

        # Project onto image plane
        projected_point = np.dot(camera_matrix, transformed_point)
        if (projected_point[2][0]):
            u = (projected_point[0][0] / projected_point[2][0])
            v = (projected_point[1][0] / projected_point[2][0])
            if (0 <= u <= image_size[0] and 0 <= v <= image_size[1]):
                image_point = np.array([u, v])

                # Draw the projected point on the image
                points.append(image_point)
                ids.append(x)
        x += 1
    # print(ids)
    return points, ids


class Pose:
    def __init__(self, data):
        self.time = data[0]
        self.T = np.array([data[1], data[2], data[3]])
        quaternion = data[4:]
        self.R = self.quaternion_to_rotation_matrix(quaternion)

    def quaternion_to_rotation_matrix(self, q):  # x, y ,z ,w
        # rot_matrix = np.array(
        #     [[1.0 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] - q[3] * q[2]), 2 * (q[3] * q[1] + q[0] * q[2])],
        #      [2 * (q[0] * q[1] + q[3] * q[2]), 1.0 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] - q[3] * q[0])],
        #      [2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[1] * q[2] + q[3] * q[0]), 1.0 - 2 * (q[0] * q[0] + q[1] * q[1])]])
        # return rot_matrix
        r = R.from_quat(q)
        Rm = r.as_matrix()
        return Rm


# 将四元数转换为旋转矩阵
data = []


def pc_generator():
    file_path = './dataset/stamped_traj_estimate.txt'
    bag_file = '../Downloads/360_v2/cp/cp_2022-02-26.bag'
    voxel_size = 0.01
    data = read_txt(file_path)
    pose = []

    pcd, time = read_pcd(bag_file)
    total_pcd = np.array(pcd[0])
    # pcd_dict = dict(zip(time, pcd))
    for i in range(0, len(data)):
        pose.append(Pose(data[i]))
    j = 0
    pointcloud = o3d.geometry.PointCloud()
    # pointcloud.points = o3d.utility.Vector3dVector(pcd[0])
    pcd_tmp = []
    for i in tqdm(range(len(time))):
        # print(time[i])
        if abs(time[i] - pose[j].time) < ERROR:
            item = generate_pcd(pcd[i], pose[j])
            # print(pcd[i].shape)
            # print(item)
            if pcd_tmp != []:
                pcd_tmp = np.concatenate((pcd_tmp, np.array(item)), axis=0)
                # print("pcd_tmp:")
                # print(pcd_tmp.shape)
            else:
                pcd_tmp = item
            if j < len(pose) - 1:
                j += 1

        if j % 10 == 9 or (j == len(pose) - 1):
            pcd_tmp = np.array(pcd_tmp)
            # print(pcd_tmp)
            pointcloud.points = o3d.utility.Vector3dVector(pcd_tmp)
            pcd_tmp = downsample_pcd(pointcloud, voxel_size=voxel_size)  # 降采样
            total_pcd = np.concatenate((total_pcd, pcd_tmp), axis=0)
            pcd_tmp = []
        if j == len(pose) - 1:
            break
    total_pointcloud = o3d.geometry.PointCloud()
    total_pcd = total_pcd.T
    total_pcd = np.concatenate((total_pcd[2].reshape([-1, 1]), -total_pcd[0].reshape([-1, 1]),
                                -total_pcd[1].reshape([-1, 1])), axis=1)
    total_pointcloud.points = o3d.utility.Vector3dVector(total_pcd)
    o3d.io.write_point_cloud("cp_sparse2.pcd", total_pointcloud)


def pc_projection():
    file_path = '../Downloads/360_v2/cp/cp_output_point_cloud_map.pcd'
    txt_path = '../Downloads/360_v2/cp/stamped_traj_estimate.txt'
    data = read_txt(txt_path)
    pose = []
    for k in range(0, len(data)):
        pose.append(Pose(data[k]))
    # Set camera parameters (example values, replace with actual values)

    camera_matrix = np.array([[286.7005816, 0, 344.58165003],
                              [0, 279.95972671, 243.8413728],
                              [0, 0, 1]])
    width = 640
    height = 480
    points = []
    cloud = o3d.io.read_point_cloud(file_path)
    # Project each 3D point onto the image plane
    i = 0
    for j in tqdm(range(0, len(pose))):
        if (j % 10 == 0):
            points, ids = project_point_cloud_to_image(cloud.points, camera_matrix, pose[j], (width, height))
            with open('output.txt', 'w') as f:

                f.write(str(i + 1) + "\t" + str(data[i][4]) + "\t" + str(data[i][5]) + "\t" + str(data[i][6]) + "\t" + str(
                    data[i][7]) + "\t"
                        + str(data[i][1]) + "\t" + str(data[i][2]) + "\t" + str(data[i][3]) + "\t" + str(
                    i + 1) + "\t" + str(i + 1) + ".jpg"
                        + "\n")
                for k in range(len(ids)):
                    f.write(str(points[k][0]) + "\t" + str(points[k][1]) + "\t" + str(ids[k]) + "\t")
                f.write("\n")
        i += 1


if __name__ == "__main__":
    pc_projection()
