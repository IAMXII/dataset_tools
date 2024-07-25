import os
import shutil

def copy_images(src_dir, dst_dir, prefix="image"):
    # 检查目标目录是否存在，如果不存在则创建
    if not os.path.exists(dst_dir):
        os.makedirs(dst_dir)

    # 遍历源目录中的所有文件
    for filename in os.listdir(src_dir):
        # 检查文件是否以指定前缀开头
        if filename.startswith(prefix):
            # 构造源文件和目标文件的完整路径
            src_file = os.path.join(src_dir, filename)
            dst_file = os.path.join(dst_dir, filename)
            # 复制文件
            shutil.copy(src_file, dst_file)
            print(f"Copied {src_file} to {dst_file}")

# 示例用法
src_directory = "../exam/result/nerf++/cp/render_test_500000"
dst_directory = "../exam/result/nerf++/cp/render"
copy_images(src_directory, dst_directory)