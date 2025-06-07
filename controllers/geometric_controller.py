import sys , os
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(base_dir)

import numpy as np
from tools import tools

class geometric_controller:
    def __init__(self, m = 2.2, g = 9.8):
        self.m = m
        self.g = g
        self.KP = np.diag([10, 10, 30])
        self.KV = np.diag([25, 25, 15])
        self.KR = np.diag([12, 12, 12])
        self.KW = np.diag([2, 2, 2])
        self.error_buffer = np.zeros((6, 1))
        self.desire_state = np.zeros((6,1))

    def calculate_output(self, r, r_dot, R, angular_vel, r_des, r_dot_des, r_ddot_des, psi_des, angular_vel_des = np.array([0, 0, 0]).reshape(-1, 1)):
        """
        根据平坦轨迹[r_des, psi_des]计算控制量, 输出为[F, u2, u3, u4]
        
        参数:
        r (np.ndarray): 3x1的位置向量
        r_dot (np.ndarray) : 3x1的位置导数向量
        R (np.ndarray) : 3x3的旋转矩阵
        angular_vel (np.ndarray): 3x1的角速度向量
        r_des (np.ndarray): 3x1的期望位置向量
        r_dot_des (np.ndarray): 3x1的期望速度向量
        r_ddot_des (np.ndarray): 3x1的期望加速度向量
        psi_des (float): 期望偏航角
        angular_vel_des (np.ndarray): 3x1的期望角速度向量, 默认全为0
        返回:
        np.ndarray: 对应的四维控制输出 [F, u2, u3, u4]
        """

        # 计算期望推力
        e_P = r - r_des
        e_V = r_dot - r_dot_des
        z_W = np.array([0, 0, 1]).reshape(-1, 1)   # 列向量
        F_des = self.KP @ e_P + self.KV @ e_V + self.m * self.g * z_W + self.m * r_ddot_des # pd + 加速度前馈
        # u_1 = F_des.T @ R @ np.array([0, 0, 1]).reshape(-1, 1)
        u_1 = F_des.T @ R @ np.array([0, 0, 1]).reshape(-1, 1)

        # 计算期望力矩
        z_B_des = F_des / (np.linalg.norm(F_des) + 1e-4)
        # z_B_des = np.array([0, 1, 1]).reshape(-1, 1)
        # z_B_des = z_B_des / np.linalg.norm(z_B_des)
        # type1
        x_C_des = np.array([np.cos(psi_des), np.sin(psi_des), 0]).reshape(-1, 1)
        y_B_des = np.cross(z_B_des.flatten(), x_C_des.flatten()).reshape(-1, 1)
        y_B_des = y_B_des / np.linalg.norm(y_B_des)
        x_B_des = np.cross(y_B_des.flatten(), z_B_des.flatten()).reshape(-1, 1)
        x_B_des = x_B_des / np.linalg.norm(x_B_des)
        # type2
        # y_C_des = np.array([-np.sin(psi_des), np.cos(psi_des), 0])
        # x_B_des = np.cross(y_C_des.flatten(), z_B_des.flatten()).reshape(-1, 1)
        # x_B_des = x_B_des / np.linalg.norm(x_B_des)
        # y_B_des = np.cross(z_B_des.flatten(), x_B_des.flatten()).reshape(-1, 1)

        R_des = np.hstack((x_B_des, y_B_des, z_B_des))
        temp_e_R = 0.5 * (R_des.T @ R - R.T @ R_des)
        e_R = tools.vee_map(temp_e_R)
        # e_R = temp_e_R @ np.array([0, 0, 1]).reshape(-1, 1)
        # e_R = np.array([0, 0, 0]).reshape(-1, 1)
        e_W = angular_vel - angular_vel_des
        tau = - self.KR @ e_R - self.KW @ e_W
        attitude_des = tools.rotation_matrix_to_euler_angles(R_des)
        self.desire_state[3:] = np.array(attitude_des).reshape(-1, 1)
        self.desire_state[:3] = r_des
        return np.vstack((u_1, tau))

def generate_random_rotation_matrix():
    """生成随机有效的3x3旋转矩阵"""
    # 使用格拉姆-施密特正交化
    A = np.random.randn(3, 3)
    Q, _ = np.linalg.qr(A)
    # 确保行列式为1（旋转而非反射）
    if np.linalg.det(Q) < 0:
        Q[:, 0] = -Q[:, 0]
    return Q

def run_test_case():
    """运行单个测试用例"""
    controller = geometric_controller()
    
    # 生成随机状态和参考轨迹
    r = np.random.randn(3, 1) * 2      # 位置 [m]
    r_dot = np.random.randn(3, 1)      # 速度 [m/s]
    R = generate_random_rotation_matrix()  # 旋转矩阵
    angular_vel = np.random.randn(3, 1) * 0.5  # 角速度 [rad/s]
    
    r_des = np.random.randn(3, 1)      # 期望位置 [m]
    r_dot_des = np.random.randn(3, 1) * 0.2  # 期望速度 [m/s]
    r_ddot_des = np.random.randn(3, 1) * 0.1  # 期望加速度 [m/s^2]
    psi_des = np.random.uniform(-np.pi, np.pi)  # 期望偏航角 [rad]
    psi_des = 0
    angular_vel_des = np.random.randn(3, 1) * 0.1  # 期望角速度 [rad/s]
    
    # 计算控制输出
    try:
        control_output = controller.calculate_output(
            r, r_dot, R, angular_vel, 
            r_des, r_dot_des, r_ddot_des, 
            psi_des, angular_vel_des
        )
        
        # 验证输出格式
        assert control_output.shape == (4, 1), f"输出形状错误: {control_output.shape}"
        
        # 检查力的大小是否合理（应接近mg）
        thrust = control_output[0, 0]
        expected_thrust = controller.m * controller.g
        print(R)
        assert abs(thrust - expected_thrust) < 20, f"推力异常: {thrust} N (预期约{expected_thrust} N)"
        
        print("测试通过!")
        print(f"控制输出: \n{control_output.round(3)}")
        return True
    except Exception as e:
        print(f"测试失败: {e}")
        return False

if __name__ == "__main__":
    # 运行10次随机测试
    num_tests = 10
    passed = 0
    R = getR(np.array([0, np.pi / 4, 0]))
    print(R)
    for i in range(num_tests):
        print(f"\n=== 测试 {i+1}/{num_tests} ===")
        if run_test_case():
            passed += 1
    
    print(f"\n测试结果: {passed}/{num_tests} 通过")

