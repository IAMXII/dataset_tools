import rosbag
import open3d as o3d
# from sensor_msgs.msg import PointCloud
# from sensor_msgs import point_cloud
import os
from geometry_msgs.msg import Point32
from argparse import ArgumentParser
import numpy as np
import cv2
from tqdm import tqdm
global i
parser = ArgumentParser()
parser.add_argument('--bag_path', '-s')
parser.add_argument('--output_path', '-o')
args = parser.parse_args()
step = 1 / 6


def read_from_bag(bag_file):
    bag = rosbag.Bag(bag_file, 'r')
    pc_data = []
    im_data = []
    time_0 = []
    time1 = 1e12
    # for topic, msg, t in bag.read_messages(topics=['/radar_enhanced_pcl']):
    #     # time.append(msg.header.stamp.to_sec())
    #     if topic == '/radar_enhanced_pcl':
    #         # points = o3d.io.read_point_cloud(msg)
    #         points = msg.points
    #         pc_data.append(points)
    for topic, msg, t in tqdm(bag.read_messages(topics=['/livox/lidar'])):
        # image = cv2.imread(msg)
        time_cur = msg.header.stamp.to_sec()
        if time_cur < time1:
            time1 = time_cur
        time_0.append(time_cur)
        # print(time_cur)
        # print(msg.points[0])
        # break
        points = msg.points
        pc_points = []
        for point in points:
            pc_points.append(np.array([point.x, point.y, point.z]))
        # print(pc_points)
        pc_data.append(pc_points)
        # print("done")
    time_0 = time_0[10:-10]
    pc_data = pc_data[10:-10]
    with open("output1.txt", 'w') as file:
        # file.write(str(time_cur) + '\n')
        for item in time_0:
            file.write(str(item) + '\n')

    i = 0
    j = 0
    time_pre = 0
    msg_pre = []
    time2 = 1e12
    print("images over")
    # print(len(time_0))
    for topic, msg, t in bag.read_messages(topics=['/rgb_cam/image_raw/compressed']):
        # image = cv2.imread(msg)
        time_cur = msg.header.stamp.to_sec()
        # print(i,"\n")
        if (time_pre < time_0[i] <= time_cur):
            if time_0[i] - time_pre < time_cur - time_0[i]:
                np_arr = np.frombuffer(msg_pre.data, np.uint8)
            else:
                np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imwrite(f"{output_folder2}/image_" + str(i+4930) + ".jpg", cv_image)
            print("Saved image" + str(i))
            # im_data.append(cv_image)
            if (i < len(time_0)-1):
                i += 1
                if time_0[i] <= time_cur and i < len(time_0)-1:
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    cv2.imwrite(f"{output_folder2}/image_" + str(i+4930) + ".jpg", cv_image)
                    i += 1
                    if time_0[i] <= time_cur and i < len(time_0)-1:
                        np_arr = np.frombuffer(msg.data, np.uint8)
                        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        cv2.imwrite(f"{output_folder2}/image_" + str(i+4930) + ".jpg", cv_image)
                        i += 1
                with open("output3.txt", 'a') as file:
                    file.write(str(time_cur) + '\n')
                    file.write(str(time_0[i - 1]))
        with open("output3.txt", 'a') as file:
            file.write(str(time_cur) + '\n')
        #     file.write(str(j) + '\n')

        time_pre = time_cur
        msg_pre = msg
        j += 1

    bag.close()
    print(len(im_data))
    return im_data, pc_data


def save(im_data, pc_data, output_folder1, output_folder2):
    # i = 0
    # for img in im_data:
    #     cv2.imwrite(f"{output_folder2}/image_" + str(i) + ".jpg", img)
    #     i += 1
    #     print("Saved image" + str(i))
    i = 0
    for points in pc_data:
        # x = []
        # y = []
        # z = []
        # for point in points:
        #     x.append(point.x)
        #     y.append(point.y)
        #     z.append(point.z)
        # points = np.array([x, y, z])
        # points = points.T
        pcd_filename = f"{output_folder1}/pointcloud_" + str(i+4930) + ".pcd"
        i = i + 1
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(pcd_filename, pointcloud)
        # pcl.save(points_array, pcd_filename)
        print(f"Saved {pcd_filename}")


if __name__ == "__main__":
    i = 0
    # bag_file = '../Downloads/360_v2/cp/cp_2022-02-26.bag'  # args.bag_path
    bag_file = args.bag_path
    bag_path = os.path.dirname(bag_file)
    if not args.output_path:
        output_folder1 = os.path.join(bag_path, 'pcds_lidar')
        output_folder2 = os.path.join(bag_path, 'images_lidar')

        if not os.path.exists(output_folder1):
            os.makedirs(output_folder1)
        if not os.path.exists(output_folder2):
            os.makedirs(output_folder2)
    else:
        output_folder1 = os.path.join(args.output_path, "pcds_lidar")
        output_folder2 = os.path.join(args.output_path, "images_lidar")
        if not os.path.exists(output_folder1):
            os.makedirs(output_folder1)
        if not os.path.exists(output_folder2):
            os.makedirs(output_folder2)
    im_data, pc_data = read_from_bag(bag_file)

    # Save point cloud data as PCD files
    save(im_data, pc_data, output_folder1, output_folder2)
    print(len(im_data))
