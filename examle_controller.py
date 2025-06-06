import numpy as np
import matplotlib.pyplot as plt
#%matplotlib inline

def dxdt(F, X, t, h=1e-2):
    assert(len(F)==len(X))
    X = np.array(X)
    K1 = np.array([f(X, t) for f in F])
    dX = h*K1/2
    K2 = np.array([f(X+dX, t+h/2) for f in F])
    dX = h*K2/2
    K3 = np.array([f(X+dX, t+h/2) for f in F])
    dX = h*K3
    K4 = np.array([f(X+dX, t+h) for f in F])

    dX = (K1 + 2*K2 + 2*K3 + K4)*h/6

    return dX, np.array([f(X, t) for f in F])

def sat(x, delta):
    return  x/delta if np.abs(x)<delta else np.sign(x)

def fal(x, alpha=0.5, delta=0.1):
    return  x/np.power(delta,1-alpha) if np.abs(x)<delta else np.power(np.abs(x), alpha)*np.sign(x)

# target signal
def v(t):
    # if t < 10:
    #     return np.sign(np.sin(0.8*t))
    # elif t < 20:
    #     return 2*(0.5*t-int(0.5*t)-0.5)
    # else:
    #     return np.sin(0.8*t)
    return 1

def v1(X, t):
    x1, x2 = X[0], X[1]
    return x2

def v2(X, t):
    x1, x2 = X[0], X[1]
    return -R*sat(x1 - v(t) + np.abs(x2)*x2/(2*R), delta)

# eso
# 极点配置
wc = 35
p = np.poly1d([-wc,-wc,-wc],True)
_, b1, b2, b3 = tuple(p.coef)

def g1(X, t):
    x1,x2,x3 = X[0], X[1], X[2]
    return x2 - b1 * (x1 - y)  # y is model output

def g2(X, t):
    x1, x2, x3 = X[0], X[1], X[2]
    return x3 - b2 * (x1 - y) + 5*u

def g3(X, t):
    x1, x2, x3 = X[0], X[1], X[2]
    return -b3 * (x1 - y)

# hidden uncertain model
def f1(X, t):
    x, y = X[0], X[1]
    return y

def f2(X, t):
    x, y = X[0], X[1]
    #return -x*x*x - x -0.2*y  + u #+ w(t)
    return -x - y + 5 * u

def w(t):
    return 0.2 * np.sign(np.cos(t))  # perturbation

R = 90  # params in sal
delta = 0.01  # params in sal
h = 0.01  # discrete time unit
T = 5  # total time
N = int(T/h)  # num of points
V = [0., 0.]  # TD signal
X = [0., 0.]  # true state
Z = [0., 0., 0.]  # ESO
u = 0  # initial control input

actual_output = []
expect_output = []
uncertain_dynamics = []
t_list = []
f_list = []

for i in range(N):
    t = i*h  # time
    t_list.append(t)

    dX, _ = dxdt([f1,f2],X,t,h)
    X = X + dX
    y = X[0]  # model output

    dV, _ = dxdt([v1,v2],V,t,h)
    V = V + dV

    dZ, _ = dxdt([g1,g2,g3],Z,t,h)
    Z = Z + dZ

    e_p = V[0] - Z[0]
    e_d = V[1] - Z[1]

    fep, fed = fal(e_p), fal(e_d)

    u = (14*fep + 49*fed - Z[2])/5

    actual_output.append(y)
    expect_output.append(V[0])
    uncertain_dynamics.append(Z[2])
    f_list.append(-X[0]-X[1])

plt.plot(t_list, uncertain_dynamics, label="f_hat")
plt.plot(t_list, f_list, label="f_des")
plt.legend()
plt.show()

plt.plot(actual_output, color='black', label='output')
plt.plot(expect_output, color='red', linestyle='--',label='expect')
plt.plot(uncertain_dynamics, color='green', linestyle='--',label='uncertain state')
plt.legend(loc='lower right')
plt.show()
