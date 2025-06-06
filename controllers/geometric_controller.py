import numpy as np

class geometic_controller:
    def __init__(self, m = 1.2, g = 9.81):
        self.m = m
        self.g = g
        self.Kp = np.diag([10, 10, 10])
        self.Kv = np.diag([1, 1, 1])
        self.Kr = np.diag([1, 1, 1])
        self.Kw = np.diag([1, 1, 1])

    def calculate_output(self, r, r_dot, r_des, r_dot_des, r_ddot_des, psi_des):
        e_p = r - r_des
        e_v = r_dot - r_dot_des
        z_w = np.array([0, 0, 1]).reshape(-1, 1)   # 列向量
        F_des =  - self.Kp @ e_p - self.Kv @ e_v + self.m * self.g * z_w + self.m * r_ddot_des
        z_B_des = F_des / np.linalg.norm(F_des)
        x_C_des = np.array([np.cos(psi_des), np.sin(psi_des), 0]).reshape(-1, 1)
        y_B_des = np.cross(z_B_des, x_C_des)
        y_B_des = y_B_des / np.linalg.norm(y_B_des)
        x_B_des = np.cross(z_B_des, y_B_des)
        x_B_des = x_B_des / np.linalg.norm(x_B_des)

