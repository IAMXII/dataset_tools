import numpy as np
from scipy.spatial.transform import Rotation as R


# 定义一个3x3的旋转矩阵
class SLAM_Operations():
    def __init__(self):
        self.R = np.eye(3)
        self.T = np.zeros(3)
        self.K = np.eye(4)  ###RT 4*4 juzhen
        self.intrinsics = np.eye(3)
        self.Q = np.zeros(4)

    def set_params(self, R=None, T=None, K=None, intrinsics=None, Q=None):
        self.R = R
        self.T = T
        self.K = K
        self.intrinsics = intrinsics
        self.Q = Q

    def R_to_Q(self):
        rotation = R.from_matrix(self.R)
        self.Q = rotation.as_quat()
        return self.Q

