import numpy as np
import math


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


def vex(x):
    """
   获取一个3x3矩阵的反对称矩阵
    :param x: 一个3x3矩阵
    :return: 其反对称矩阵
    """
    return np.array([x[2][1], x[0][2], x[1][0]])


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
