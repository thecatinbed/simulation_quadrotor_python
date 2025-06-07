import math

import matplotlib.pyplot as plt
import numpy as np
from model import quadrotors_model as QModel
from model import quadrotors_parameters as QPara
from draw import draw  # 导入draw模块
from draw import draw3d  # 导入draw3d模块
from tools import tools, trajectory_generator
from controllers import geometric_controller

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

def main():
    quadrotors_model = QModel.QuadrotorsModel()  # 创建四旋翼模型对象
    controller = geometric_controller.geometric_controller(QPara.m, QPara.g)
    traj_generator = trajectory_generator(QPara.dt, trajectory_generator.STRAIGHT)
    t = []  # 创建时间列表，初始时间为0
    y_list = [] 
    t_now = 0  # 当前时间初始化为0
    des_traj = []

    ctrl_buffer = np.zeros(6)
    f_list = [0]
    flag = False
    while t_now < 60:
        y_now = quadrotors_model.measurement
        y_list.append(y_now)  # 记录当前模型的输出
        attitude = y_now[6:9]
        angular_vel = np.array(y_now[9:12]).reshape(-1, 1) 
        r = np.array(y_now[:3]).reshape(-1, 1)
        r_dot = np.array(y_now[3:6]).reshape(-1, 1)
        R = tools.getR(attitude).reshape(3, 3)
        if not flag and abs(quadrotors_model.measurement[2]) < abs(traj_generator.takeoff_height):
            des_traj = traj_generator.takeoff()
        else:
            flag = True
            des_traj = traj_generator.get_trajectory()

        r_des = np.array(des_traj[:3]).reshape(-1, 1)
        psi_des = 0
        r_dot_des = np.array(des_traj[4:]).reshape(-1, 1)

        # r_des = np.array([0,0,1]).reshape(-1, 1)
        # psi_des = 0
        # r_dot_des = np.array([0,0,0]).reshape(-1, 1)

        f_tau = controller.calculate_output(r, r_dot, R, angular_vel, r_des, r_dot_des, np.array([0,0,0]).reshape(-1,1), psi_des)    
        f_list.append(f_tau[0])
        print(f_tau)
        # 计算油门
        throttle = calculate_throttle(f_tau)
        # 更新模型
        quadrotors_model.step(throttle, t_now)

        
        t.append(t_now)
        t_now += QPara.dt  # 增加时间步长
        ctrl_out = np.hstack([r_des.flatten(), [psi_des, 0, 0]])
        ctrl_buffer = np.vstack((ctrl_buffer, ctrl_out))
    
    t.append(t_now)
    plt.plot(t, f_list)
    draw.draw(t,ctrl_buffer, quadrotors_model.measurement_buffer, 0, 0, "euler")
    # draw3d.draw3d( ctrl_buffer, quadrotors_model.measurement_buffer, 0, "euler3D")

if __name__ == '__main__':
    main()  # 调用main()函数进行程序执行
    print("ok")