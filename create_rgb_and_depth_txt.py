import numpy as np
import cv2

# with open('rgb.txt', 'w') as f:
#     for i in range(1205):
#         f.write(str(i) + ' ' + "rgb/image_" + str(i) + ".jpg" + "\n")
# with open('depth.txt', 'w') as f:
#     for i in range(1205):
#         f.write(str(i) + ' ' + "depth/depth_" + str(i) + ".png" + "\n")

# with open('../dataset/nyl/gt_pose/gt_odom_nyl.txt', 'r') as f:
#     lines = f.readlines()
#     lines = lines[2101:4416]
# with open("gt.txt", 'w') as f:
#     for i, line in enumerate(lines):
#         line = line.strip()
#         line = line.split(' ')
#         line[0] = str(i)
#         f.write(
#             line[0] + " " + line[1] + " " + line[2] + " " + line[3] + " " + line[4] + " " + line[5] + " " + line[
#                 6] + " " + line[7] + "\n")

with open('../dataset/cp/groundtruth.txt', 'r') as f:
    lines = f.readlines()
with open('groundtruth.txt', 'w') as f:
    for i in range(len(lines)):
        if i % 2 == 0:
            line = lines[i].strip()
            line = line.split(' ')
            line[0] = str(i/2)
            f.write(
                line[0] + " " + line[1] + " " + line[2] + " " + line[3] + " " + line[4] + " " + line[5] + " " + line[
                    6] + " " + line[7] + "\n")

