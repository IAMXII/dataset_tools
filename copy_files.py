### copy some files to another directory
import os
import shutil
from tqdm import tqdm
from argparse import ArgumentParser


def sort_by_number(filename):
    return int(''.join(filter(str.isdigit, filename)))


parser = ArgumentParser()
parser.add_argument('--input_path', '-s')
parser.add_argument('--output_path', '-o')
args = parser.parse_args()

file = os.listdir(args.input_path)
file_path = [os.path.join(args.input_path, f) for f in file]
file = sorted(file, key=sort_by_number)
file_path = sorted(file_path, key=sort_by_number)

# for i in range(2851):
#     if i % 10 == 0:
#         file_o_path = os.path.join(args.output_path, "images_" + str(int(i / 10)) + ".jpg")
#         shutil.copy(file_path[i], file_o_path)
#         print(f'Copied: {file_o_path}')
for i in range(701):
    file_o_path = os.path.join(args.output_path, "depth_" + str(int(i)) + ".png")
    shutil.copy(file_path[i], file_o_path)
    print(f'Copied: {file_o_path}')
