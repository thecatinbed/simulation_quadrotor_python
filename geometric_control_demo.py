import math

import matplotlib.pyplot as plt
import numpy as np
from model import quadrotors_model as QModel
from model import quadrotors_parameters as QPara
from draw import draw  # 导入draw模块
from draw import draw3d  # 导入draw3d模块
from tools import tools, trajectory_generator
from controllers import geometric_controller

def main():
    quadrotors_model = QModel.QuadrotorsModel()  # 创建四旋翼模型对象
    controller = geometric_controller.geometric_controller(QPara.m, QPara.g)
    traj_generator = trajectory_generator(QPara.dt, trajectory_generator.CIRCULAR)
    t = []  # 创建时间列表，初始时间为0
    y_list = []
    des_traj = []
    t_now = 0  # 当前时间初始化为0

    ctrl_buffer = np.zeros(6)
    error_buffer = np.zeros(6)
    takeoff_flag = False    # 起飞完成标志
    while t_now < 60:
        # 记录当前模型的状态
        y_now = quadrotors_model.measurement
        y_list.append(y_now)
        # 提取无人机的姿态
        attitude = y_now[6:9]
        angular_vel = np.array(y_now[9:12]).reshape(-1, 1) 
        r = np.array(y_now[:3]).reshape(-1, 1)
        r_dot = np.array(y_now[3:6]).reshape(-1, 1)
        R = tools.getR(attitude).reshape(3, 3)
        # 获取参考轨迹
        if not takeoff_flag and abs(quadrotors_model.measurement[2]) < abs(traj_generator.takeoff_height):
            # step 1： 起飞
            des_traj = traj_generator.takeoff()
        else:
            # step 2： 跟踪轨迹
            takeoff_flag = True
            des_traj = traj_generator.get_trajectory()
        # 提取参考轨迹
        r_des = np.array(des_traj[:3]).reshape(-1, 1)
        psi_des = 0
        r_dot_des = np.array(des_traj[4:]).reshape(-1, 1)
        # 计算控制量
        f_tau = controller.calculate_output(r, r_dot, R, angular_vel, r_des, r_dot_des, np.array([0,0,0]).reshape(-1,1), psi_des)
        print("t: {}s   f:{:.2f}  u2:{:.6f}  u3:{:.6f}  u4:{:.6f}".format(t_now, f_tau[0][0], f_tau[1][0], f_tau[2][0], f_tau[3][0]))
        # 计算油门
        throttle = tools.calculate_throttle(f_tau, QPara.thrust_matrix, QPara.Cr)
        # 更新模型
        quadrotors_model.step(throttle, t_now)
        # 记录轨迹与误差
        t.append(t_now)
        t_now += QPara.dt  # 增加时间步长
        ctrl_out = controller.desire_state.flatten()
        ctrl_buffer = np.vstack((ctrl_buffer, ctrl_out))
        error_out = np.hstack([(r-r_des).flatten(), controller.desire_state.flatten()[3:]-attitude])
        error_buffer = np.vstack((error_buffer, error_out))
    t.append(t_now) # 保持长度一致
    # 绘图
    draw.draw(t,ctrl_buffer, quadrotors_model.measurement_buffer, 1, 0, "euler")
    draw.draw_error(t, error_buffer, "error")
    draw3d.draw3d( ctrl_buffer, quadrotors_model.measurement_buffer, 0, "euler3D")
    plt.show()

if __name__ == '__main__':
    main()  # 调用main()函数进行程序执行
    print("ok")