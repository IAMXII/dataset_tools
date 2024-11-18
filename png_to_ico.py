from ast import parse
from PIL import Image
import os
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-i", "--input", required=True, type=str, help="input png file")
parser.add_argument("-o", "--output", required=False, type=str, help="output ico file")
args = parser.parse_args()
ico_path = os.path.dirname(args.input)
ico = os.path.basename(args.input).split(".")[0] + ".ico"
def png_to_ico(input_file, output_file, sizes=[16, 32, 48, 64, 128, 256]):
    """
    将 PNG 文件转换为 ICO 文件。

    :param input_file: 输入的 PNG 文件路径
    :param output_file: 输出的 ICO 文件路径
    :param sizes: ICO 文件支持的尺寸列表（默认包括常见图标尺寸）
    """
    try:
        # 打开 PNG 文件
        with Image.open(input_file) as img:
            # 检查是否为 RGBA 模式，如果不是则转换
            if img.mode != 'RGBA':
                img = img.convert('RGBA')

            # 保存为 ICO 文件，支持多尺寸
            img.save(output_file, format='ICO', sizes=[(size, size) for size in sizes])

        print(f"成功将 {input_file} 转换为 {output_file}")
    except Exception as e:
        print(f"转换失败: {e}")


if __name__ == "__main__":
    # 用户输入文件路径

    output_path = os.path.join(ico_path, ico)

    # 确保文件存在
    if not os.path.isfile(args.input):
        print("输入的 PNG 文件不存在，请检查路径。")
    else:
        # 调用转换函数
        png_to_ico(args.input, output_path)