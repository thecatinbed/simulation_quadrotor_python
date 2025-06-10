import numpy as np
import math

def rotation_matrix_to_euler_angles(R):
    """
    将3x3旋转矩阵转换为欧拉角(ZYX顺序，即偏航-俯仰-滚转)

    参数:
    R (np.ndarray): 3x3旋转矩阵

    返回:
    tuple: (偏航yaw, 俯仰pitch, 滚转roll)，单位为弧度
    """
    # 确保输入是有效的3x3矩阵
    R = np.asarray(R, dtype=np.float64)
    if R.shape != (3, 3):
        raise ValueError("输入矩阵必须是3x3的旋转矩阵")

    # 计算俯仰角pitch (绕Y轴旋转)
    # 使用atan2而非asin以获得更稳定的结果
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6  # 奇异情况检测

    if not singular:
        # 非奇异情况
        x = np.arctan2(R[2, 1], R[2, 2])  # 滚转roll (绕X轴)
        y = np.arctan2(-R[2, 0], sy)  # 俯仰pitch (绕Y轴)
        z = np.arctan2(R[1, 0], R[0, 0])  # 偏航yaw (绕Z轴)
    else:
        # 奇异情况 (俯仰角接近±90°)
        x = np.arctan2(-R[1, 2], R[1, 1])  # 滚转roll (绕X轴)
        y = np.arctan2(-R[2, 0], sy)  # 俯仰pitch (绕Y轴)
        z = 0  # 偏航yaw (任意值，设为0)

    return (x, y, z)  # 返回顺序: 滚转, 俯仰, 偏航

def calculate_throttle(f_tau, thrust_matrix, Cr):
    ang_v_motors_2 = np.dot(np.linalg.inv(thrust_matrix), f_tau).flatten()
    ang_v_motors = np.zeros([4])
    for i in range(4):
        if ang_v_motors_2[i] < 0:
            ang_v_motors[i] = 0
        else:
            ang_v_motors[i] = math.sqrt(ang_v_motors_2[i])
    throttle = ang_v_motors / Cr
    return  throttle

def integral(t, x_1, x_2, size):
    """
    积分工具函数
    :param t: 采样时间
    :param x_1: 上一时刻的值
    :param x_2: 当前时刻的值
    :param size: 用于积分的数组大小
    :return: 与积分对象相同大小的数组
    """
    if size == 1:
        tm = t #采样时间
        x = (x_1 + x_2) / 2 #平均值
        y1 = x * tm
    elif size == 2:
        tm = np.array([t, t])
        x = np.add(x_1, x_2) / 2
        y1 = np.multiply(x, tm) #逐元素相乘
    elif size == 3:
        tm = np.array([t, t, t])
        x = np.add(x_1, x_2) / 2
        y1 = np.multiply(x, tm)
    elif size == 9:
        tm = np.array([t, t, t, t, t, t, t, t, t])
        x = np.add(x_1, x_2) / 2
        y1 = np.multiply(x, tm)
    return y1


def derivation(t, x_1, x_2, size):
    """
      导数工具函数
    :param t: 采样时间
    :param x_1: 上一时刻的值
    :param x_2: 当前时刻的值
    :param size: 用于求导的数组大小
    :return: 与导数对象相同大小的数组
    """
    if size == 1:
        tm = t
        x = x_2 - x_1
        y2 = x / tm
    elif size == 2:
        tm = np.array([t, t])
        x = np.array([x_2[0] - x_1[0], x_2[1] - x_1[1]])
        y2 = np.divide(x, tm)
    elif size == 3:
        tm = np.array([t, t, t])
        x = np.array([x_2[0] - x_1[0], x_2[1] - x_1[1], x_2[2] - x_1[2]])
        y2 = np.divide(x, tm)
    return y2


def sat_gd(u, a, size):
    """
  饱和函数
    :param u: 操作对象
    :param a: 饱和值
    :param size: 用于求导的数组大小。0：标量；其他：数组y
    """
    if size == 0:
        if u <= -a:
            return -a
        elif u >= a:
            return a
        else:
            return u
    else:
        u_max = 0
        for i in range(u.size):
            if abs(u[i]) > u_max:
                u_max = abs(u[i])
        if u_max <= a:
            return u
        else:
            return a * np.divide(u, u_max)

def saturation_fuc(x, lower=-1, upper=1):
    for i in range(len(x)):
        if x[i] < lower:
            x[i] = lower
        elif x[i] > upper:
            x[i] = upper
    return x


def vex(x):
    """
   获取一个3x3矩阵的反对称矩阵
    :param x: 一个3x3矩阵
    :return: 其反对称矩阵
    """
    return np.array([x[2][1], x[0][2], x[1][0]])

def vee_map(rotation_matrix):
        """
        计算3x3反对称旋转矩阵的vee映射, 得到对应的三维向量
        
        参数:
        rotation_matrix (np.ndarray): 3x3的反对称旋转矩阵
        
        返回:
        np.ndarray: 对应的三维向量
        """
        # # 验证矩阵是否为3x3
        # if rotation_matrix.shape != (3, 3):
        #     raise ValueError("输入矩阵必须是3x3的")
        
        # # 验证矩阵是否为反对称矩阵（A^T = -A）
        # if not np.allclose(rotation_matrix.T, -rotation_matrix):
        #     raise ValueError("输入矩阵必须是反对称矩阵")
        
        # 提取向量分量（反对称矩阵的特定元素）
        w = np.array([
            rotation_matrix[2, 1],  # ω₁ = M₂₃ = -M₃₂
            rotation_matrix[0, 2],  # ω₂ = M₃₁ = -M₁₃
            rotation_matrix[1, 0]   # ω₃ = M₁₂ = -M₂₁
        ]).reshape(-1, 1)
        
        return w

def normalizeWithGrad(x, xd):
    xSqrNorm = x[0] ** 2 + x[1] ** 2 + x[2] ** 2
    xNorm = np.sqrt(xSqrNorm)
    xNor = x / xNorm
    xNord = (xd - x * (np.dot(x, xd) / xSqrNorm)) / xNorm
    return xNor, xNord


def getR(attitude):
    """
      计算旋转矩阵
    :param attitude: 当前姿态
    :return: 当前旋转矩阵x
    """
    phi = attitude[0]
    theta = attitude[1]
    psi = attitude[2]
    sph = math.sin(phi)
    cph = math.cos(phi)
    st = math.sin(theta)
    ct = math.cos(theta)
    sps = math.sin(psi)
    cps = math.cos(psi)
    #计算R（5.9）
    R = np.array([ct * cps, cps * st * sph - sps * cph, cps * st * cph + sps * sph,
                  ct * sps, sps * st * sph + cps * cph, sps * st * cph - cps * sph,
                  -st, sph * ct, cph * ct])
    return R


def getApsi(psi):
    """
   计算4.2中的A_psi矩阵
    :param psi: 当前偏航角
    :return: 4.2中的A_psi矩阵
    """
    # 11.7
    sp = math.sin(psi)
    cp = math.cos(psi)
    R_psi = np.array([cp, -sp, sp, cp]).reshape(2, 2)
    x = np.array([0, 1, -1, 0]).reshape(2, 2)
    Apsi = np.dot(R_psi, x)
    return Apsi
