from tools import tools as tools
from model import quadrotors_parameters as Param

h = Param.dt

class two_order_model:
    def __init__(self):
        self.y = 0
        self.y_last = 0
        self.y_dot = 0
        self.y_dot_last = 0

    def step(self, u):
        self.y_last = self.y
        self.y_dot_last = self.y_dot
        self.y = self.y_last + h * (self.y_dot_last)
        self.y_dot = self.y_dot_last + h * (5 * u - self.y_dot_last - self.y_last)

        return (self.y, self.y_dot)
