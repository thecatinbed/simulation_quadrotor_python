import math

import matplotlib.pyplot as plt
import numpy as np
from model import quadrotors_model as QModel
from model import quadrotors_parameters as QPara
from controllers import ADRC_controller
from draw import draw  # 导入draw模块
from draw import draw3d  # 导入draw3d模块
from tools import tools, trajectory_generator

def calculate_throttle(f_tau):
    ang_v_motors_2 = np.dot(np.linalg.inv(QPara.thrust_matrix), f_tau).flatten()
    ang_v_motors = np.zeros([4])
    for i in range(4):
        if ang_v_motors_2[i] < 0:
            ang_v_motors[i] = 0
        else:
            ang_v_motors[i] = math.sqrt(ang_v_motors_2[i])
    throttle = ang_v_motors / QPara.Cr
    return  throttle

def get_trajectory(type,t):
    r = 3
    if type == 1 :
        if t <= 10:
            position = (0, 0, 1)
        elif t <= 20:
            position = (0, 0, 1)
        elif t <= 35:
            position = (r * math.cos(2 * math.pi / 30 * (t-20)) - r, r * math.sin(2 * math.pi / 30 * (t-20)) , 1)
        elif t <= 40:
            position = (r * math.cos(2 * math.pi / 30 * (35-20)) - r,r * math.sin(2 * math.pi / 30 * (35-20)), 1)
        else:
            position = (r * math.cos(2 * math.pi / 30 * (35-20)) - r,r * math.sin(2 * math.pi / 30 * (35-20)), 0)
    elif type == 0:
        if t <= 10:
            position = (0, 0, 10)
        elif t <= 20:
            position = (1, 1, 10)
        elif t <= 30:
            position = (2, 3, 10)
        elif t <= 40:
            position = (4, 2, 10)
        elif t <= 50:
            position = (1, 1, 10)
        elif t <= 60:
            position = (1, 1, 0)
    elif type == 2:
        position = (r * math.cos(2 * math.pi / 20 * (t)) - r, r * math.sin(2 * math.pi / 20 * (t)), 1)
    elif type == 3:
        position = (math.cos(2 * math.pi / 20 * (t)) + 2, math.sin(2 * math.pi / 20 * (t)) + 2, 10)
    return position

def main():
    quadrotors_model = QModel.QuadrotorsModel()  # 创建四旋翼模型对象
    t = []  # 创建时间列表，初始时间为0
    y_list = []
    y_hat_list = []
    t_now = 0  # 当前时间初始化为0

    psi_des_list = []
    psi_w0 = 10 / 0.3  # 观测器带宽, w0 = 10 / ts, ts为调节时间
    psi_wc = psi_w0 / 4
    psi_b0 = 9  # 1/QPara.J_zz
    psi_controller = ADRC_controller.ADRC_Controller(QPara.dt,
                                                     [psi_w0 * 3, 3 * psi_w0 * psi_w0, psi_w0 * psi_w0 * psi_w0],
                                                     psi_b0, psi_wc, True)

    theta_des_list = []
    theta_w0 = 10 / 0.2  # 观测器带宽, w0 = 10 / ts, ts为调节时间
    theta_wc = theta_w0 / 4
    theta_b0 = 20  # 1/QPara.J_yy
    theta_controller = ADRC_controller.ADRC_Controller(QPara.dt,
                                                     [theta_w0 * 3, 3 * theta_w0 * theta_w0, theta_w0 * theta_w0 * theta_w0],
                                                     theta_b0, theta_wc, True)
    phi_des_list = []
    phi_w0 = 10 / 0.2  # 观测器带宽, w0 = 10 / ts, ts为调节时间
    phi_wc = phi_w0 / 4
    phi_b0 = 20  # 1/QPara.J_xx
    phi_controller = ADRC_controller.ADRC_Controller(QPara.dt,
                                                     [phi_w0 * 3, 3 * phi_w0 * phi_w0, phi_w0 * phi_w0 * phi_w0],
                                                     phi_b0, phi_wc, True)

    h_des_list = []
    h_w0 = 10 / 0.2  # 观测器带宽, w0 = 10 / ts, ts为调节时间
    h_wc = phi_w0 / 4
    h_b0 = -1 / QPara.m # -1/m
    h_controller = ADRC_controller.ADRC_Controller(QPara.dt,
                                                     [h_w0 * 3, 3 * h_w0 * h_w0, h_w0 * h_w0 * h_w0],
                                                     h_b0, h_wc, True)


    horizonal_w0 = 10 / 2
    horizonal_wc = horizonal_w0 / 3.3 #3.3
    horizonal_b0 = 1

    x_des_list = []
    x_controller = ADRC_controller.ADRC_Controller(QPara.dt,
                                                   [horizonal_w0 * 3, 3 * (horizonal_w0**2), (horizonal_w0 ** 3)],
                                                   horizonal_b0, horizonal_wc, True)

    y_des_list = []
    y_controller = ADRC_controller.ADRC_Controller(QPara.dt,
                                                   [horizonal_w0 * 3, 3 * (horizonal_w0**2), (horizonal_w0 ** 3)],
                                                   horizonal_b0, horizonal_wc, True)

    ctrl_buffer = np.zeros(6)
    while t_now < 60:
        x_des = 3   # 3 * math.cos(2 * math.pi / 30 * t_now) - 3
        y_des = 3   # 3 * math.sin(2 * math.pi / 30 * t_now)
        h_des = 5
        # 跟踪微分器柔化启动信号
        x_v,_ = x_controller.trackingdifferentiator.get_signal(x_des)
        y_v,_ = y_controller.trackingdifferentiator.get_signal(y_des)
        h_v,_ = h_controller.trackingdifferentiator.get_signal(h_des)
        psi_v, _ = psi_controller.trackingdifferentiator.get_signal(0)
        # 外环线性控制率
        U1 = x_controller.get_control_output_u_3d(x_v)
        U2 = y_controller.get_control_output_u_3d(y_v)
        u1 = h_controller.get_control_output_u_3d(h_v)
        #h_controller.u = u1 - QPara.g / h_controller.b0
        matrix_b_inverse = np.multiply(np.array([[math.cos(psi_v), math.sin(psi_v)],[math.sin(psi_v), -math.cos(psi_v)]]), -horizonal_b0/(u1 / QPara.m))
        U = np.dot(matrix_b_inverse, np.array([[U1],[U2]]))
        theta_des, phi_des = float(U[0]), float(U[1])

        theta_v,_ = theta_controller.trackingdifferentiator.get_signal(theta_des)
        phi_v,_ = phi_controller.trackingdifferentiator.get_signal(phi_des)
        # 内环线性控制率
        u4 = psi_controller.get_control_quantity_u_3d(psi_v)
        u3 = theta_controller.get_control_quantity_u_3d(theta_v)
        u2 = phi_controller.get_control_quantity_u_3d(phi_v)
        
        f_tau = np.array([u1, u2, u3, u4])
        #tools.sat_gd(np.array([u1, u2, u3, u4]),10,1)
        # 计算油门
        throttle = calculate_throttle(f_tau)
        # 更新模型
        quadrotors_model.step(throttle, t_now)
        # 更新观测器
        psi_hat = psi_controller.update_obserber(quadrotors_model.measurement[8])
        theta_hat = theta_controller.update_obserber(quadrotors_model.measurement[7])
        phi_hat = phi_controller.update_obserber(quadrotors_model.measurement[6])
        h_hat = h_controller.update_obserber(quadrotors_model.measurement[2])
        y_hat = y_controller.update_obserber(quadrotors_model.measurement[1])
        x_hat = x_controller.update_obserber(quadrotors_model.measurement[0])

        y_list.append(quadrotors_model.measurement)  # 记录当前模型的输出
        y_hat_list.append(psi_hat)  # 记录预测的输出

        t.append(t_now)
        t_now += QPara.dt  # 增加时间步长
        ctrl_buffer = np.vstack((ctrl_buffer, np.array([x_des,y_des,h_des,phi_des,theta_des,0])))

    plt.plot(t, [measurement[8] for measurement in y_list], label="psi_real")
    plt.plot(t, [y[0] for y in y_hat_list], label="psi_predict")
    plt.grid(True)
    plt.legend()
    t.append(t_now)
    draw.draw(t,ctrl_buffer, quadrotors_model.measurement_buffer, 0, 0, "euler")
    draw3d.draw3d( ctrl_buffer, quadrotors_model.measurement_buffer, 0, "euler3D")

if __name__ == '__main__':
    main()  # 调用main()函数进行程序执行
    print("ok")