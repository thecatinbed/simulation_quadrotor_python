import math

from controllers import ADRC_controller
import matplotlib.pyplot as plt
from model import quadrotors_parameters as QParam

h = 0.01
T = QParam.dt
myTD = ADRC_controller.ADRC_Controller(T,[1,1,1],1,1,True).trackingdifferentiator
t_now = 0
y_des_list = []
t_list = []
y_pre_list = []
y_d_des = 0
while t_now <= 10:
    if t_now == 0:
        rdes = 0
    else:
        rdes = 1
    rdes = math.sin(2*math.pi/ 10 * t_now)
    y_d_des = 2*math.pi/10*math.cos(t_now)
    y_pre, y_d = myTD.get_signal(rdes)
    print(y_d, y_d_des)
    y_pre_list.append(y_pre)
    y_des_list.append(rdes)
    t_list.append(t_now)
    t_now += T

plt.plot(t_list, y_des_list, label="y_des")
plt.plot(t_list, y_pre_list, label="y_pre")
plt.show()