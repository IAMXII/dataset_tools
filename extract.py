import os
import tarfile

# 数据集路径
dataset_dir = "/home/liuwei/mnt/instant_vggt_dataset/TartanAir_preprocessed"
output_dir = "/home/liuwei/mnt/instant_vggt_dataset/TartanAir_unpacked"

os.makedirs(output_dir, exist_ok=True)

for file_name in os.listdir(dataset_dir):
    if file_name.endswith(".tar.gz"):
        tar_path = os.path.join(dataset_dir, file_name)
        folder_name = file_name.replace(".tar.gz", "")
        folder_path = os.path.join(output_dir, folder_name)
        os.makedirs(folder_path, exist_ok=True)

        print(f"解压 {file_name} → {folder_path}")
        with tarfile.open(tar_path, "r:gz") as tar:
            tar.extractall(path=folder_path)

print("全部解压完成！")
