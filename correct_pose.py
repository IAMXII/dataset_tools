import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.interpolate import interp1d


def load_tum_trajectory(file_path):
    """读取TUM格式轨迹"""
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith("#") or len(line.strip()) == 0:
                continue
            parts = line.strip().split()
            if len(parts) == 8:
                t, tx, ty, tz, qx, qy, qz, qw = map(float, parts)
                data.append([t, tx, ty, tz, qx, qy, qz, qw])
    return np.array(data)


def normalize_time(timestamps):
    """归一化时间戳到[0, 1]"""
    t_min, t_max = np.min(timestamps), np.max(timestamps)
    # return (timestamps - t_min) / (t_max - t_min)
    return (timestamps - t_min) / 5428

def interpolate_trajectory(traj, num_points=40000):
    """
    插值轨迹到指定点数
    traj: [N, 8] (t, tx, ty, tz, qx, qy, qz, qw)
    """
    timestamps = traj[:, 0]
    norm_t = normalize_time(timestamps)

    # 新的均匀采样点
    new_t = np.linspace(0, 0.3682756, num_points)

    # 平移部分插值
    interp_funcs = [interp1d(norm_t, traj[:, i], kind='linear') for i in range(1, 4)]
    new_trans = np.vstack([f(new_t) for f in interp_funcs]).T

    # 旋转部分插值 (SLERP)
    rotations = R.from_quat(traj[:, 4:8])  # (qx, qy, qz, qw)
    slerp = Slerp(norm_t, rotations)
    new_rots = slerp(new_t).as_quat()

    # 拼接结果
    new_traj = np.hstack([new_t[:, None], new_trans, new_rots])
    return new_traj


if __name__ == "__main__":
    file_path = "/home/liuwei/mnt/experiment_RAL/MonoGS/traj_livgs0_ori_radar.txt"  # 输入你的TUM轨迹路径
    traj = load_tum_trajectory(file_path)
    new_traj = interpolate_trajectory(traj, num_points=400)

    # 保存结果
    np.savetxt("/home/liuwei/mnt/experiment_RAL/MonoGS/trajectory_interpolated.txt", new_traj, fmt="%.6f")
    print("插值完成，结果已保存到 trajectory_interpolated.tum")
