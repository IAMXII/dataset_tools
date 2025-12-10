import numpy as np

def load_tum_trajectory(file_path):
    """
    读取 TUM 格式：timestamp tx ty tz qx qy qz qw
    返回：[(t, 4x4_T)]
    """
    traj = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.strip() == "" or line[0] == "#":
                continue
            data = list(map(float, line.strip().split()))
            t = data[0]
            tx, ty, tz = data[1:4]
            qx, qy, qz, qw = data[4:8]

            # convert to 4x4
            T = quat_tran_to_matrix(tx, ty, tz, qx, qy, qz, qw)
            traj.append((t, T))
    return traj


def quat_tran_to_matrix(tx, ty, tz, qx, qy, qz, qw):
    """ quaternion + translation → 4x4 SE3 """
    q = np.array([qw, qx, qy, qz])  # w x y z
    # normalize
    q = q / np.linalg.norm(q)

    w, x, y, z = q
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),  1 - 2*(x*x + y*y)]
    ])
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def matrix_to_tum(T, timestamp):
    """ 4x4 → TUM line """
    R = T[:3, :3]
    t = T[:3, 3]
    qw, qx, qy, qz = rot_to_quat(R)
    return f"{int(timestamp)} {t[0]} {t[1]} {t[2]} {qx} {qy} {qz} {qw}"


def rot_to_quat(R):
    """ 3x3 rotation matrix → (qw, qx, qy, qz) """
    qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
    qx = (R[2,1] - R[1,2]) / (4*qw)
    qy = (R[0,2] - R[2,0]) / (4*qw)
    qz = (R[1,0] - R[0,1]) / (4*qw)
    return qw, qx, qy, qz


def subsample_traj(traj, target_num=200):
    """ 等间隔抽取 200 帧 """
    total = len(traj)
    if total <= target_num:
        return traj
    idx = np.linspace(0, total - 1, target_num).astype(int)
    return [traj[i] for i in idx]


def convert_lidar_to_cam(traj, lidar2cam):
    """
    LiDAR 世界位姿 T_world_lidar
    Camera 位姿为：T_world_cam = T_world_lidar * lidar2cam
    """
    new_traj = []
    for t, T_wl in traj:
        T_wc = T_wl @ lidar2cam
        new_traj.append((t, T_wc))
    return new_traj


def normalize_trajectory(traj):
    """
    把第一帧变成 Identity
    T'_i = inv(T_0) * T_i
    """
    T0 = traj[0][1]
    T0_inv = np.linalg.inv(T0)
    new_traj = []
    for t, T in traj:
        Tn = T0_inv @ T
        new_traj.append((t, Tn))
    return new_traj


def save_tum(traj, out_path):
    with open(out_path, "w") as f:
        for t, T in traj:
            line = matrix_to_tum(T, t)
            f.write(line + "\n")


def process_lidar_tum_to_cam(tum_path, out_path, lidar2cam, num_frames=200):
    traj = load_tum_trajectory(tum_path)
    traj = subsample_traj(traj, num_frames)
    traj = convert_lidar_to_cam(traj, lidar2cam)
    traj = normalize_trajectory(traj)
    save_tum(traj, out_path)
    print(f"Saved camera TUM trajectory to: {out_path}")


# =======================
# 使用示例
# =======================
if __name__ == "__main__":
    lidar2cam = np.array([
        [-0.01783068, -0.99978912, 0.01020197, -0.0803292],
        [-0.02988329, -0.00966631, -0.99950683, 0.10752965],
        [0.99939454, -0.01812658, -0.0297048, -0.06024161],
        [0., 0., 0., 1.]
    ])  # 示例矩阵，请替换

    process_lidar_tum_to_cam(
        tum_path="/home/liuwei/mnt/dataset/cp/gt_odom.txt",
        out_path="/home/liuwei/mnt/vggt_dataset/cp/001/cp_lvsm/camera_traj.txt",
        lidar2cam=lidar2cam,
        num_frames=200
    )
