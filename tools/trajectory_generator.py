from enum import Enum
import math

class trajectory_generator():
    CIRCULAR = 1
    STRAIGHT = 2
    def __init__(self, dt, trajectory_type: Enum, takeoff_height = 3):
        self.trajectory_type = trajectory_type
        self.trajectory = [0, 0, 0, 0, 0, 0, 0] # [x, y, z, psi, vx, vy, vz]  
        self.dt = dt
        self.t = 0
        self.takeoff_vel = 0.1
        self.takeoff_height = 3
        self.circular_radius = 3
        self.angular_vel = 3 / self.circular_radius

    def takeoff(self):
        self.trajectory[2] += self.takeoff_vel * self.dt
        self.trajectory[-1] = self.takeoff_vel
        return self.trajectory

    def get_trajectory(self):
        if self.trajectory_type == self.CIRCULAR: # 圆心为（0, radius）, 轨迹方程为 x^2 + (y-radius)^2 = radius^2; x = radius * cos(omega * t - pi / 2), y = radius * (sin(omega * t - pi / 2) + 1) 
            self.trajectory[0] = self.circular_radius * math.cos(self.angular_vel * self.t - math.pi / 2)
            self.trajectory[1] = self.circular_radius * (math.sin(self.angular_vel * self.t - math.pi / 2) + 1)
            self.trajectory[2] = self.takeoff_height
            self.trajectory[3] = 0
            self.trajectory[4] = - self.angular_vel * self.circular_radius * math.sin(self.angular_vel * self.t - math.pi / 2)
            self.trajectory[5] = self.angular_vel * self.circular_radius * math.cos(self.angular_vel * self.t - math.pi / 2)
            self.trajectory[6] = 0
        else:
            self.trajectory[0] = 0
            self.trajectory[1] = 1
            self.trajectory[2] = self.takeoff_height
            self.trajectory[3] = 0
            self.trajectory[4] = 0
            self.trajectory[5] = 0
            self.trajectory[6] = 0
        self.t += self.dt
        return self.trajectory