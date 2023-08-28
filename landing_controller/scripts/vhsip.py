import matplotlib
matplotlib.use('TkAgg')
import numpy as np
from scipy.linalg import expm
import matplotlib.pyplot as plt

m = 40
k = 1000
d = 2*np.sqrt(m*k)
g = 9.81
l0 = 0.7

dt = 0.002
N = 1000

Z = np.zeros([2, N]) * np.nan
X0 = np.zeros([2, N]) * np.nan
X1 = np.zeros([2, N]) * np.nan
X2 = np.zeros([2, N]) * np.nan
X3 = np.zeros([2, N]) * np.nan
X4 = np.zeros([2, N]) * np.nan
u0 = 0
u1 = -0.1
u2 = 0.1
u3 = -0.2
u4 = 0.2

time = np.arange(0, N)*dt

Z[0, 0] = -g*0.2
Z[1, 0] = 0

X0[0,0] = 1.5
X0[1,0] = 0
X1[0,0] = 1.5
X1[1,0] = 0
X2[0,0] = 1.5
X2[1,0] = 0
X3[0,0] = 1.5
X3[1,0] = 0
X4[0,0] = 1.5
X4[1,0] = 0

Az = np.array([[-d/m, -k/m],[1,0]])
for i in range(0, N-1):
    Z[:, i] = expm(Az * time[i])@ Z[:, 0]
    zdd = Az[0, :] @ Z[:, i]
    omega2 = (-g+zdd)/(Z[1, i]+l0)

    Ax = np.array([[0, omega2],[0, 0]])
    Bx = np.array([[0], [-omega2]])

    Ax_bar = np.eye(2) + Ax * dt
    Bx_bar = Bx * dt

    X0[:, i + 1] = ( np.reshape(Ax_bar @ X0[:, i], [2, 1]) + Bx_bar * u0).flatten()
    X1[:, i + 1] = ( np.reshape(Ax_bar @ X1[:, i], [2, 1]) + Bx_bar * u1).flatten()
    X2[:, i + 1] = ( np.reshape(Ax_bar @ X2[:, i], [2, 1]) + Bx_bar * u2).flatten()
    X3[:, i + 1] = ( np.reshape(Ax_bar @ X3[:, i], [2, 1]) + Bx_bar * u3).flatten()
    X4[:, i + 1] = ( np.reshape(Ax_bar @ X4[:, i], [2, 1]) + Bx_bar * u4).flatten()

plt.figure()
plt.plot(time, Z[1, :]+l0)
plt.plot(time, X0[1, :])
plt.plot(time, X1[1, :])
plt.plot(time, X2[1, :])
plt.plot(time, X3[1, :])
plt.plot(time, X4[1, :])
plt.legend(['z', 'x0', 'x1', 'x2', 'x3', 'x4'])
plt.show()

plt.figure()
plt.plot(time,  Z[0, :])
plt.plot(time, X0[0, :])
plt.plot(time, X1[0, :])
plt.plot(time, X2[0, :])
plt.plot(time, X3[0, :])
plt.plot(time, X4[0, :])
plt.legend(['dz', 'dx0', 'dx1', 'dx2', 'dx3', 'dx4'])
plt.show()