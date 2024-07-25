###
import cv2
import numpy as np
import os
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--source_path", "-s")
args = parser.parse_args()

def sort_by_number(filename):
    return int(''.join(filter(str.isdigit, filename)))


def apply_depth_color_map(depth_value, max_depth):
    """
    根据深度值将其映射到蓝色到红色之间的颜色。
    """
    normalized_depth = depth_value / max_depth  # 将深度值归一化到0-1之间
    normalized_depth = 1 - normalized_depth
    color_value = int(normalized_depth * 255)
    return cv2.applyColorMap(np.array([[color_value]], dtype=np.uint8), cv2.COLORMAP_JET)[0][0]


def overlay_depth_on_image(color_image, depth_image, max_depth, radius):
    """
    将深度信息以颜色标注的方式叠加到彩色图像上，每个点有一定的半径。
    """
    result_image = color_image.copy()

    rows, cols = depth_image.shape
    for i in range(rows):
        for j in range(cols):
            depth_value = depth_image[i, j]
            if depth_value > 0:  # 只处理有深度信息的点
                color = apply_depth_color_map(depth_value, max_depth)
                cv2.circle(result_image, (j, i), radius, color.tolist(), -1)

    return result_image


def process_images(color_image_dir, depth_image_dir, output_dir, radius=3):
    """
    批量处理图像，将深度信息叠加到彩色图像上。
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    depth_path = os.listdir(depth_image_dir)
    image_path = os.listdir(color_image_dir)
    depth_paths = sorted(depth_path, key=sort_by_number)
    image_paths = sorted(image_path, key=sort_by_number)

    for i in range(628, len(depth_path)):
        depth_image_path = os.path.join(depth_image_dir, depth_paths[i])
        color_image_path = os.path.join(color_image_dir, image_paths[i])

        # 检查深度图像是否存在
        if not os.path.exists(depth_image_path):
            print(f"深度图像 {depth_image_path} 不存在，跳过...")
            continue

        # 读取彩色图像和深度图像
        color_image = cv2.imread(color_image_path)
        depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

        # 获取深度图像的最大深度值
        max_depth = np.max(depth_image)

        # 将深度信息叠加到彩色图像上
        result_image = overlay_depth_on_image(color_image, depth_image, max_depth, radius)

        # 保存结果图像
        output_path = os.path.join(output_dir, image_paths[i])
        cv2.imwrite(output_path, result_image)
        print(f"保存结果图像到 {output_path}")


# 示例用法
if __name__ == '__main__':
    color_image_dir = os.path.join(args.source_path, "images_lidar_2")
    depth_image_dir = os.path.join(args.source_path, "depths_lidar_2")
    output_dir = os.path.join(args.source_path, "ouput")
    radius = 0  # 可以调整半径大小

    process_images(color_image_dir, depth_image_dir, output_dir, radius)
