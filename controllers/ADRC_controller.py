import math

from tools import tools
import numpy as np

class Extended_State_Observer:
    def __init__(self, L, T = 0.01):
        self.z1_hat_last = 0
        self.z1_hat_now = 0
        self.z2_hat_last = 0
        self.z2_hat_now = 0
        self.z3_hat_last = 0
        self.z3_hat_now = 0
        self.beta_1 = L[0]
        self.beta_2 = L[1]
        self.beta_3 = L[2]
        self.init_flag = True
        self.T = T

    def update_3d(self, h, u, y, b0):
        if not self.init_flag:
            self.init_flag = True
            self.z1_hat_now = u
            self.z2_hat_now = 0
            self.z3_hat_now = 0
        else:
            self.z1_hat_last = self.z1_hat_now
            self.z2_hat_last = self.z2_hat_now
            self.z3_hat_last = self.z3_hat_now
            self.z1_hat_now = self.z1_hat_last + self.T * (self.z2_hat_last + self.beta_1 * (y - self.z1_hat_last))
            self.z2_hat_now = self.z2_hat_last + self.T * (self.z3_hat_last + b0 * u + self.beta_2 * (y - self.z1_hat_last))
            self.z3_hat_now = self.z3_hat_last + self.T * (self.beta_3 * (y - self.z1_hat_last))

    def update_2d(self, h, u, y, b0):
        self.z1_hat_last = self.z1_hat_now
        self.z2_hat_last = self.z2_hat_now
        self.z1_hat_now = self.z1_hat_last + self.T * (-self.beta_1 * self.z1_hat_last + self.z2_hat_last + b0 * u + self.beta_1 * y)
        self.z2_hat_now = self.z2_hat_last + self.T * (-self.beta_2 * self.z1_hat_last + self.beta_2 * y)

class TrackingDifferentiator:
    def __init__(self, h, r, T):
        # 跟踪器参数
        self.h = h  # 此处h为滤波因子, 并非采样步长
        self.r = r  # 速度因子,决定跟踪速度
        self.d = r * h
        self.d0 = self.d * h
        self.T = T
        # 保存跟踪器数据
        self.x1_last = 0
        self.x1_now = 0
        self.x2_last = 0
        self.x2_now = 0
        self.init_flag = True

    def sign(self, y):
        if y > 0:
            return 1
        elif y == 0:
            return 0
        else:
            return -1

    def get_fhan(self, u, x1, x2):
        y = x1 - u + self.h * x2
        a0 = math.sqrt(self.d * self.d + 8 * self.r * abs(y))
        if abs(y) <= self.d0:
            a = x2 + y / self.h
        else:
            a = x2 + 0.5 * (a0 - self.d) * self.sign(y)
        if abs(a) <= self.d:
            fhan = -self.r * a / self.d
        else:
            fhan = -self.r * self.sign(a)
        return fhan

    def get_signal(self, u):
        if self.init_flag:
            self.x1_last = self.x1_now
            self.x2_last = self.x2_now
            self.x1_now = self.x1_last + self.x2_last * self.T
            self.x2_now = self.x2_last + self.get_fhan(u, self.x1_last, self.x2_last) * self.T
        else:
            self.x1_now = u
            self.x2_now = 0
            self.init_flag = True
        return (self.x1_now, self.x2_now)

class Lineral_Controller:
    def __init__(self, wc):
        self.kp = wc * wc
        self.kv = 2 * wc

    def get_control_output_u0(self, ep, ed):
        return self.kp * ep + self.kv * ed

class ADRC_Controller:
    def __init__(self, T, L, b0, wc, td_flag):   # h为步长, 即微分时的时间;L矩阵为观测器的参数
        self.u = 0
        self.b0 = b0
        self.T = T
        self.observer = Extended_State_Observer(L)
        self.controller = Lineral_Controller(wc)
        self.td_flag = td_flag                  # 是否启动跟踪微分器（TD）
        if td_flag:
            self.trackingdifferentiator = TrackingDifferentiator(0.01, 5, T)
        self.r_last = 0

    def get_control_output_u_3d(self, r):#r为输入， y为输出
        ep = tools.sat_gd(np.array([ep]),0.13,1)[0]       # 0.13
        if self.td_flag:
            ed = self.trackingdifferentiator.x2_now - self.observer.z2hat_now
        else:
            ed = (r - self.r_last) / self.T
        ed = tools.sat_gd(np.array([ed]), 0.2, 1)[0]      # 0.2
        u0 = self.controller.get_control_output_u0(ep, ed)
        self.u = (u0 - self.observer.z3hat_now) / self.b0
        self.r_last = r
        return self.u
    
    def get_control_output_u_3d(self, r, y):#r为输入， y为输出
        ep = tools.sat_gd(np.array([ep]),0.13,1)[0]       # 0.13
        if self.td_flag:
            ed = self.trackingdifferentiator.x2_now - self.observer.z2hat_now
        else:
            ed = (r - self.r_last) / self.T
        ed = tools.sat_gd(np.array([ed]), 0.2, 1)[0]      # 0.2
        u0 = self.controller.get_control_output_u0(ep, ed)
        self.u = (u0 - self.observer.z3hat_now) / self.b0
        self.r_last = r
        self.update_obserber(y)
        return self.u

    def update_obserber(self, y):
        self.observer.update_3d(self.T, self.u, y, self.b0)  # 拓张状态观测器观测一次结果
        return  (self.observer.z1hat_now, self.observer.z2hat_now, self.observer.z3hat_now)

    def get_control_quantity_u_2d(self, r, y):
        self.observer.update_2d(self.T, self.u, y, self.b0)  
        u0 = self.controller.get_control_quantity_u0(r - self.observer.z1hat_now, 0)
        self.u = (u0 - self.observer.z2hat_now) / self.b0
        return self.u

    def init_start_value(self, value):
        self.r_last = value

class Complete_Controller:
    def __init__(self):
        pass
