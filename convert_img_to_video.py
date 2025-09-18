import cv2
import os

def sort_by_number(filename):
    return int(''.join(filter(str.isdigit, filename)))

def images_to_video(image_folder, output_path="output.mp4", fps=30):
    # 按文件名排序
    images = sorted(os.listdir(image_folder), key=sort_by_number)
    images = [img for img in images if img.lower().endswith(('.png', '.jpg', '.jpeg'))]

    if not images:
        raise ValueError("文件夹中没有找到图像！")

    # 读取第一张图像来获取尺寸
    first_frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = first_frame.shape

    # 定义视频编码器 (mp4v / XVID / MJPG 等)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    for img_name in images:
        img_path = os.path.join(image_folder, img_name)
        frame = cv2.imread(img_path)
        if frame is None:
            print(f"跳过无效图像: {img_path}")
            continue
        video.write(frame)

    video.release()
    print(f"视频已保存到 {output_path}")


if __name__ == "__main__":
    # 修改路径和参数
    images_to_video(
        image_folder="/home/liuwei/mnt/experiment_RAL/MonoGS/results/mnt_dataset/enhanced_radar/render/after_opt/images",  # 图片所在文件夹
        output_path="output1.mp4",          # 输出视频路径
        fps=30                             # 帧率
    )
