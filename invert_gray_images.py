import os
from PIL import Image, ImageOps

def invert_gray_images(input_dir, output_dir):
    """
    将 input_dir 文件夹中的所有灰度图颜色翻转，并保存到 output_dir。
    文件名保持不变。
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 支持的图片扩展名
    exts = ['.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff']

    for filename in os.listdir(input_dir):
        if any(filename.lower().endswith(ext) for ext in exts):
            input_path = os.path.join(input_dir, filename)
            output_path = os.path.join(output_dir, filename)

            try:
                # 打开图像并转为灰度
                img = Image.open(input_path).convert("L")
                inverted_img = ImageOps.invert(img)  # 灰度反色
                inverted_img.save(output_path)
                print(f"[OK] {filename} -> {output_path}")
            except Exception as e:
                print(f"[ERROR] {filename}: {e}")

if __name__ == "__main__":
    input_dir = "/home/liuwei/mnt/instant_vggt_dataset/sky/val/labels"      # 输入文件夹路径
    output_dir = "/home/liuwei/mnt/instant_vggt_dataset/sky/val/labels_1"    # 输出文件夹路径
    invert_gray_images(input_dir, output_dir)
