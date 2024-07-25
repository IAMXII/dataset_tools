from PIL import Image
import os
import numpy as np

def find_max_depth_point(folder_path):
    max_depth = -1
    max_depth_point = None

    for filename in os.listdir(folder_path):
        if filename.endswith(".png") or filename.endswith(".jpg"):
            image_path = os.path.join(folder_path, filename)
            # Load the image
            depth_image = Image.open(image_path)
            # Convert to numpy array
            depth_array = np.array(depth_image)
            # Find maximum depth value and its position
            max_depth_in_image = np.max(depth_array)
            max_depth_pos = np.unravel_index(np.argmax(depth_array), depth_array.shape)
            # Compare with overall maximum
            if max_depth_in_image > max_depth:
                max_depth = max_depth_in_image
                max_depth_point = max_depth_pos

    return max_depth_point, max_depth

# Example usage:
folder_path = "../dataset/cp/depths_lidar_2"
max_depth_point, max_depth_value = find_max_depth_point(folder_path)
print(f"The maximum depth point is located at {max_depth_point} with depth value {max_depth_value}")