import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


class MPCController:
    def __init__(self, dt=0.01, N=10, mass=1.0, gravity=9.81):
        # 系统参数
        self.dt = dt  # 采样时间
        self.N = N  # 预测时域长度
        self.mass = mass  # 无人机质量
        self.gravity = gravity  # 重力加速度

        # 定义状态误差权重矩阵 Q（对角矩阵）
        self.Q = ca.diagcat(
            10, 10, 10,  # 位置误差 (x,y,z) 的权重
            0, 0, 0  # 速度误差 (vx,vy,vz) 的权重
        )

        # 定义控制输入权重矩阵 R
        self.R = ca.diagcat(
            0.1,  # 推力权重
            10,  # 滚转角权重
            10,  # 俯仰角权重
            0.1  # 偏航角权重
        )

        # 创建优化问题
        self.opti = ca.Opti()
        self.solver_initialized = False
        self.setup_mpc_problem()

    def setup_mpc_problem(self):
        """设置MPC优化问题"""
        # 状态变量: [位置(x,y,z), 速度(vx,vy,vz)]
        self.x = self.opti.variable(6, self.N + 1)

        # 控制输入: [总推力, 滚转角, 俯仰角, 偏航角]
        self.u = self.opti.variable(4, self.N)

        # 参考轨迹
        self.x_ref = self.opti.parameter(6, self.N + 1)

        # 初始状态
        self.x0 = self.opti.parameter(6)

        # 系统动态模型
        for k in range(self.N):
            # 状态转移
            x_next = self.quadrotor_dynamics(self.x[:, k], self.u[:, k])
            self.opti.subject_to(self.x[:, k + 1] == x_next)

        # # 约束条件
        self.opti.subject_to(self.x[:, 0] == self.x0)  # 初始状态约束
        #
        # # 控制输入约束
        self.opti.subject_to(self.u[0, :] >= 0.1 * self.mass * self.gravity)  # 最小推力
        self.opti.subject_to(self.u[0, :] <= 10.0 * self.mass * self.gravity)  # 最大推力
        self.opti.subject_to(self.u[1, :] >= -np.deg2rad(30))  # 最小滚转角
        self.opti.subject_to(self.u[1, :] <= np.deg2rad(30))  # 最大滚转角
        self.opti.subject_to(self.u[2, :] >= -np.deg2rad(30))  # 最小俯仰角
        self.opti.subject_to(self.u[2, :] <= np.deg2rad(30))  # 最大俯仰角
        self.opti.subject_to(self.u[3, :] >= -np.pi)  # 最小偏航角
        self.opti.subject_to(self.u[3, :] <= np.pi)  # 最大偏航角

        # 目标函数
        obj = 0
        for k in range(self.N + 1):
            # 状态跟踪误差
            state_error = self.x[:, k] - self.x_ref[:, k]
            obj += ca.mtimes([state_error.T, self.Q, state_error])

        # for k in range(self.N):
        #     # 控制输入代价
        #     obj += 0.1 * ca.sumsqr(self.u[:, k])
        #
        #     # 控制输入变化率代价
        #     if k > 0:
        #         obj += 0.01 * ca.sumsqr(self.u[:, k] - self.u[:, k - 1])

        self.opti.minimize(obj)

        # 设置求解器
        p_opts = {"expand": True}
        s_opts = {"max_iter": 100, "print_level": 0}
        self.opti.solver('ipopt', p_opts, s_opts)
        self.solver_initialized = True

    def quadrotor_dynamics(self, x, u, dt = 0.01):
        """四旋翼动力学模型（无小角度假设）"""
        # 提取状态和控制
        p = x[0:3]  # 位置
        v = x[3:6]  # 速度
        thrust = u[0]  # 总推力
        phi = u[1]  # 滚转角
        theta = u[2]  # 俯仰角
        psi = u[3]  # 偏航角

        # 重力向量
        g = ca.vertcat(0, 0, -self.gravity)

        # 从欧拉角构建旋转矩阵 (ZYX顺序: 偏航-俯仰-滚转)
        R_roll = ca.vertcat(
            ca.horzcat(1, 0, 0),
            ca.horzcat(0, ca.cos(phi), ca.sin(phi)),
            ca.horzcat(0, -ca.sin(phi), ca.cos(phi))
        )

        R_pitch = ca.vertcat(
            ca.horzcat(ca.cos(theta), 0, ca.sin(theta)),
            ca.horzcat(0, 1, 0),
            ca.horzcat(-ca.sin(theta), 0, ca.cos(theta))
        )

        R_yaw = ca.vertcat(
            ca.horzcat(ca.cos(psi), ca.sin(psi), 0),
            ca.horzcat(-ca.sin(psi), ca.cos(psi), 0),
            ca.horzcat(0, 0, 1)
        )

        # 总旋转矩阵 (机体到惯性)
        R = R_yaw @ R_pitch @ R_roll

        # 机体坐标系下的推力向量
        F_body = ca.vertcat(0, 0, thrust)

        # 将推力转换到惯性坐标系
        F_inertial = R @ F_body

        # 位置导数 (速度)
        p_dot = v

        # 速度导数 (加速度)
        v_dot = F_inertial / self.mass + g

        # 状态导数
        x_dot = ca.vertcat(p_dot, v_dot)

        # 欧拉积分 (离散化)
        x_next = x + dt * x_dot

        return x_next

    def solve(self, x0, x_ref):
        """求解MPC问题，返回最优控制输入"""
        # 检查参数形状
        assert x0.shape == (6,), f"初始状态形状错误: 期望(6,), 得到{x0.shape}"
        assert x_ref.shape == (6, self.N + 1), f"参考轨迹形状错误: 期望(6,{self.N + 1}), 得到{x_ref.shape}"

        # 设置参数值
        self.opti.set_value(self.x0, x0)
        self.opti.set_value(self.x_ref, x_ref)

        try:
            sol = self.opti.solve()
            # 获取求解统计信息
            stats = sol.stats()
            print(f"优化求解状态: {stats['return_status']}")
            print(f"迭代次数: {stats['iter_count']}")
            print(f"目标函数值: {sol.value(self.opti.f)}")
            u_opt = sol.value(self.u)
            return u_opt[:, 0]  # 返回第一个控制输入
        except Exception as e:
            print(f"MPC求解失败: {e}")

            # 尝试获取未设置的参数信息
            try:
                for param in self.opti.parameters():
                    if not self.opti.is_value_set(param):
                        print(f"未设置值的参数: {param.name()}, 形状: {param.shape}")
            except:
                pass

            # 返回默认控制输入
            return np.array([self.mass * self.gravity, 0, 0, 0])


# 简单的四旋翼模拟器，用于测试MPC控制器
def simulate_quadrotor(controller, x0, x_ref, sim_time=5.0, dt=0.01):
    """模拟四旋翼无人机在MPC控制下的轨迹"""
    t = np.arange(0, sim_time, dt)
    n_steps = len(t)

    # 存储状态和控制
    x_history = np.zeros((n_steps + 1, 6))
    u_history = np.zeros((n_steps, 4))
    x_ref_history = np.zeros((n_steps + 1, 6))

    x_history[0] = x0

    for i in range(n_steps):
        radius = 3
        T = 5
        phi = - ca.pi / 2
        traj_x = radius * ca.cos(2 * ca.pi * i * dt / T + phi)
        traj_y = radius * (ca.sin(2 * ca.pi * i * dt / T + phi) + 1)
        traj_z = 0
        x_ref = np.array([traj_x,traj_y,traj_z,0,0,0])
        x_ref_history[i] = x_ref
        # 当前状态
        x_current = x_history[i]

        # 生成参考轨迹
        x_ref_traj = np.tile(x_ref, (controller.N + 1, 1)).T

        # 打印调试信息
        if i % 10 == 0:
            print(f"仿真步骤 {i}/{n_steps}, 当前状态: {x_current}")

        # 求解MPC问题
        # if i % 10 == 0:
        u_opt = controller.solve(x_current, x_ref_traj)
        u_history[i] = u_opt
        # 手动悬停
        # u_hover = np.array([controller.mass * controller.gravity + 0.1, 0, 0, 0])
        # u_opt = u_hover
        # u_history[i] = u_hover

        x_history [i+1] = controller.quadrotor_dynamics(x_current, u_opt, dt=dt).full().flatten()
    x_ref_history[-1] = x_ref
    return t, x_ref_history, x_history, u_history


# 示例使用
if __name__ == "__main__":
    # 创建MPC控制器
    controller = MPCController(dt=0.01, N=10, mass=1.0, gravity=9.81)

    # 初始状态 [x, y, z, vx, vy, vz]
    x0 = np.array([0, 0, 0, 0, 0, 0])

    # 目标状态
    x_ref = np.array([5, 3, 2, 0, 0, 0])  # 悬停在(5, 3, 2)位置

    # 模拟
    print("开始仿真...")
    t, x_ref_history, x_history, u_history = simulate_quadrotor(controller, x0, x_ref, sim_time=10.0)

    # 绘制结果
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 2, 1)
    plt.plot(t, x_history[:-1, 0], label='x')
    plt.plot(t, x_history[:-1, 1], label='y')
    plt.plot(t, x_history[:-1, 2], label='z')
    plt.plot(t, x_ref_history[:-1,0], color='r', linestyle='--', label='x_ref')
    plt.plot(t, x_ref_history[:-1,1], color='g', linestyle='--', label='y_ref')
    plt.plot(t, x_ref_history[:-1,2], color='b', linestyle='--', label='z_ref')
    plt.legend()
    plt.title('Position')

    plt.subplot(2, 2, 2)
    plt.plot(t, x_history[:-1, 3], label='vx')
    plt.plot(t, x_history[:-1, 4], label='vy')
    plt.plot(t, x_history[:-1, 5], label='vz')
    plt.title('Velocity')
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.plot(t, u_history[:, 0], label='Thrust')
    plt.title('Control Input - Thrust')
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.plot(t, np.rad2deg(u_history[:, 1]), label='Roll (deg)')
    plt.plot(t, np.rad2deg(u_history[:, 2]), label='Pitch (deg)')
    plt.plot(t, np.rad2deg(u_history[:, 3]), label='Yaw (deg)')
    plt.title('Control Input - Attitudes')
    plt.legend()

    plt.tight_layout()
    plt.show()