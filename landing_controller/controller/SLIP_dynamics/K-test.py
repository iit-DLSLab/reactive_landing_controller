import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

if __name__ == '__main__':
    samples = 2000
    time = np.linspace(0,0.5,samples)

    m = 12.
    max_set_time = 0.5

    k_min = m * (8/max_set_time)**2


    p = np.zeros_like(time)
    v = np.zeros_like(time)

    p_min = -0.10
    V0 = [-0.1, -1.0, -7.0, -10.0]
    legend = []
    fig = plt.figure(0)
    for v0 in V0:
        k = k_min
        legend.append('v0:'+str(v0)+', k:'+str(k))
        eig = -np.sqrt(k / m)
        for i in range(0,samples):
            t = time[i]
            p[i] = np.exp(eig*t)*t*v0
            v[i] = np.exp(eig * t + 1) * v0

        plt.plot(time, p)

    fig = plt.figure(1)
    for v0 in V0:
        k = m * (v0 / (p_min * np.exp(1))) ** 2
        legend.append('v0:' + str(v0) + ', k:' + str(k))
        eig = -np.sqrt(k / m)
        for i in range(0, samples):
            t = time[i]
            p[i] = np.exp(eig * t) * t * v0
            v[i] = np.exp(eig * t + 1) * v0

        plt.plot(time, p)

    plt.legend(legend)
