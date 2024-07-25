import rosbag
import open3d as o3d
# from sensor_msgs.msg import PointCloud
# from sensor_msgs import point_cloud
import os
from geometry_msgs.msg import Point32
from argparse import ArgumentParser
import numpy as np

parser = ArgumentParser()
parser.add_argument('--bag_path', '-s', required=True)
parser.add_argument('--output_path', '-o')
args = parser.parse_args()
    


def read_point_cloud_from_bag(bag_file):
    bag = rosbag.Bag(bag_file, 'r')
    pc_data = []

    for topic, msg, t in bag.read_messages(topics=['/radar_enhanced_pcl']):
        if topic == '/radar_enhanced_pcl':
            # points = o3d.io.read_point_cloud(msg)
            points = msg.points
            pc_data.append(points)
           
    bag.close()
    return pc_data


def save_pcd(pc_data, output_folder):
    global i

    for points in pc_data:
        x = []
        y = []
        z = []
        for point in points:
            x.append(point.x)
            y.append(point.y)
            z.append(point.z)
        points = np.array([x,y,z])
        points = points.T
        pcd_filename = f"{output_folder}/pointcloud_"+str(i)+".pcd"
        i = i + 1
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(pcd_filename, pointcloud)
        # pcl.save(points_array, pcd_filename)
        print(f"Saved {pcd_filename}")


if __name__ == "__main__":
    i=5940
    bag_file = args.bag_path
    bag_path = os.path.dirname(bag_file)
    if not args.output_path:
        output_folder = os.path.join(bag_path, 'pcds')
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
    else:
        output_folder = os.path.join(args.output_path, "pcds")
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
    # Read point cloud data from ROS Bag file
    pc_data = read_point_cloud_from_bag(bag_file)

    # Save point cloud data as PCD files
    save_pcd(pc_data, output_folder)
