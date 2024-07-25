import open3d as o3d
from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument('--pcd',required=True)
args = parser.parse_args()
pcd = o3d.io.read_point_cloud(args.pcd)
num_points = pcd.points
print(num_points)