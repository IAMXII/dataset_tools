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
bag = rosbag.Bag('../dataset/day_10/kth_day_10_d455t-002.bag', 'r')
for topic, msg, t in bag.read_messages(topics=['/d455t/color/image_raw']):
    time_cur = msg.header.stamp.to_sec()
    np_arr = np.frombuffer(msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imwrite(f"image_0.jpg", cv_image)
    break
