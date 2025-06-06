from tools import tools as tools
from model import quadrotors_parameters as Param

h = Param.dt

class one_order_model:
    def __init__(self):
        self.y = 0
        self.y_last = 0

    def step(self, u):
        self.y_last = self.y
        self.y = self.y_last + h * (5/3 * u - 1/3 * self.y_last)

        return self.y
