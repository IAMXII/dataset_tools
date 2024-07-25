import os
import cv2
import numpy as np

camera_matrix = np.array([[332.232689, 0.0, 333.058485],[0.0, 332.644823, 240.998586], [0.0,0.0,1.0]])
dist_coeff = np.array([0.011770,-0.041851, 0.000701, -0.000319, 0.0])


image_folder = "../Downloads/360_v2/cp/origin"

image_files = [f for f in os.listdir(image_folder) if f.endswith(('.jpg', '.png', '.jpeg'))]
i = 0
for img_file in image_files:

    img_path = os.path.join(image_folder, img_file)

    img = cv2.imread(img_path)

    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeff)

    undistorted_path = os.path.join(image_folder, "undistorted_images", str(i)+".jpg")

    cv2.imwrite(undistorted_path, undistorted_img)
    i+=1
print("over")
# 332.232689, 332.644823, 333.058485, 240.998586
