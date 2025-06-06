from enum import Enum
import math

CIRCULAR = 1
STRAIGHT = 2

class trajectory_generator():
    def __init__(self, dt, type: Enum):
        self.trajectory_type = type
        self.trajectory = [0, 0, 0, 0]  # 微分平坦输出: [x, y, z, psi]
        self.dt = dt
        self.t = 0
        self.takeoff_vel = 1
        self.hover_height = 0
        self.circular_radius = 3
        self.angular_vel = 3 / self.circular_radius

    def takeoff(self):
        self.trajectory[2] += self.takeoff_vel * self.dt
        self.hover_height = self.get_trajectory[2]
        return self.trajectory

    def get_trajectory(self):
        if self.trajectory_type == CIRCULAR: # 圆心为（0, radius）, 轨迹方程为 x^2 + (y-radius)^2 = radius^2; x = radius * cos(omega * t - pi / 2), y = radius * (sin(omega * t - pi / 2) + 1) 
            self.trajectory[0] = self.circular_radius * math.cos(self.angular_vel * self.t - math.pi / 2)
            self.trajectory[1] = self.circular_radius * (math.sin(self.angular_vel * self.t - math.pi / 2) + 1)
            self.trajectory[2] = self.hover_height
            self.trajectory[3] = 0
        else:
            self.trajectory[0] = self.circular_radius * math.cos(self.angular_vel * self.t - math.pi / 2)
            self.trajectory[1] = self.circular_radius * (math.sin(self.angular_vel * self.t - math.pi / 2) + 1)
            self.trajectory[2] = self.hover_height
            self.trajectory[3] = 0
        self.t += self.dt
        return self.trajectory