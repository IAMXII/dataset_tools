



import numpy as np

def load_tum(file):
    traj = []
    with open(file) as f:
        for line in f:
            if line.startswith("#") or len(line.strip()) == 0:
                continue
            data = line.split()
            traj.append([float(x) for x in data])
    return np.array(traj)

def save_tum(file, traj):
    with open(file, "w") as f:
        for row in traj:
            f.write(" ".join(map(str, row)) + "\n")

# 加载原始轨迹
traj = load_tum("/home/liuwei/mnt/experiment_RAL/MonoGS/trajectory_interpolated.txt")

# 参数
interval = 30   # 每隔多少帧触发一次抖动
duration = 20    # 每次抖动持续多少帧
sigma = 0.04     # 抖动幅度（米）

for start in range(0, traj.shape[0], interval):
    end = min(start + duration, traj.shape[0])
    noise = np.random.normal(0, sigma, size=3)
    for i in range(start, end):
        alpha = (i - start) / duration  # 从0到1
        traj[i, 1:4] += alpha * noise   # 平滑进入抖动


# 保存结果
save_tum("/home/liuwei/mnt/experiment_RAL/MonoGS/trajectory_interpolated_jitter.txt", traj)
