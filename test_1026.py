import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
import open3d as o3d
import os
# 指定bag文件路径
bag_path = '/Volumes/LiuweiSSD/dataset/rosbag/hku1.bag'
pointcloud_topic = '/livox/lidar'
directory = os.path.dirname(bag_path)
directory = os.path.dirname(directory)
output_path = os.path.join(directory, 'hku1')
os.makedirs(os.path.join(directory, 'hku1'), exist_ok=True)
# 打开并读取bag文件
bag = bagreader(bag_path)

# 将点云主题数据提取为CSV文件
pointcloud_csv = bag.message_by_topic(pointcloud_topic)

# 读取CSV数据
df = pd.read_csv(pointcloud_csv)
for timestamp, frame in df.groupby('Time'):
    # 提取每帧的x、y、z列数据
    points = frame[['x', 'y', 'z']].values

    # 转换为Open3D格式并保存为PCD文件
    point_cloud_o3d = o3d.geometry.PointCloud()
    point_cloud_o3d.points = o3d.utility.Vector3dVector(points)

    # 根据时间戳保存文件

    o3d.io.write_point_cloud(os.path.join(os.path.join(directory, 'hku1'),f"pointcloud_{timestamp}.pcd"), point_cloud_o3d)
    break