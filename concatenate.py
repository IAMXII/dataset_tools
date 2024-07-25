import cv2 as cv
import numpy as np
import os
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--source_path", "-s", equired=True)
args = parser.parse_args()
mask = [480, 1280, 3]
img1 = []
img2 = []
result = []
folder_path1 = os.path.join(args.source_path, "gt")
folder_path2 = os.path.join(args.source_path, "renders")
for filename in os.listdir(folder_path1):
    img_1 = cv.imread(os.path.join(folder_path1, filename))
    img_1 = np.array(img_1)
    img1.append(img_1)
for filename in os.listdir(folder_path2):
    img_2 = cv.imread(os.path.join(folder_path2, filename))
    img_2 = np.array(img_2)
    img2.append(img_2)
for i in range(len(img1)):
    result.append(np.concatenate((img1[i], img2[i]), axis=1))
    output_path = os.path.join(args.source_path, "result/{}.jpg".format(i))
    cv.imwrite(output_path, result[i])
