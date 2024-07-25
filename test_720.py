import numpy as np
from scipy.spatial.transform import Rotation as R


def read_pose_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    poses = []
    for line in lines:
        numbers = list(map(float, line.strip().split()))
        if len(numbers) == 12:
            poses.append(numbers)
        else:
            print(f"Invalid pose length in line: {line}")
    return poses


def convert_pose_with_index(index, pose_12):
    rotation_matrix = np.array(pose_12[:9]).reshape((3, 3))
    rotation_matrix[0] = pose_12[:3]
    rotation_matrix[1] = pose_12[4:7]
    rotation_matrix[2] = pose_12[8:11]
    translation_vector = np.array([pose_12[3], pose_12[7], pose_12[11]])

    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()  # 输出格式为 (x, y, z, w)

    pose_8 = np.hstack((translation_vector, quaternion))
    pose_with_index = np.insert(pose_8, 0, index)
    return pose_with_index


def write_poses_to_file(poses_9, output_file_path):
    with open(output_file_path, 'w') as file:
        for pose in poses_9:
            line = ' '.join(map(str, pose))
            file.write(line + '\n')


def main(input_file_path, output_file_path):
    poses_12 = read_pose_from_file(input_file_path)
    poses_9 = [convert_pose_with_index(index, pose) for index, pose in enumerate(poses_12)]
    write_poses_to_file(poses_9, output_file_path)


# 示例使用
input_file_path = 'stamped_traj_estimate1.txt'  # 包含12个数的3x4矩阵的文件
output_file_path = 'output.txt'  # 转换后的9个数的文件

main(input_file_path, output_file_path)
