import os
import glob

def get_image_names(directory):
    # 使用glob模块获取指定目录下的所有图片文件
    image_files = glob.glob(os.path.join(directory, '*.jpg')) + glob.glob(os.path.join(directory, '*.png')) + glob.glob(os.path.join(directory, '*.jpeg'))
    image_files = sorted(image_files)
    # 提取文件名（不含后缀）并存入列表
    image_names = [os.path.splitext(os.path.basename(file))[0] for file in image_files]

    return image_names

def save_to_txt(image_names, txt_file):
    # 将图片名写入txt文件
    with open(txt_file, 'w') as f:
        for name in image_names:
            f.write(name + '\n')

if __name__ == "__main__":
    # 指定图片所在的目录
    image_directory = '../Downloads/360_v2/kitty_sequences/8'

    # 获取图片名列表
    image_names = get_image_names(image_directory)

    # 指定要保存的txt文件路径
    txt_file_path = 'image_names8.txt'

    # 将图片名保存到txt文件中
    save_to_txt(image_names, txt_file_path)

    print(f"Image names have been saved to {txt_file_path}")

