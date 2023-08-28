import matplotlib as mpl
mpl.use('TkAgg')
import numpy as np
np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed number of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)
import matplotlib.pyplot as plt
from controller.SLIP_dynamics.SLIP_dynamics_lib import SLIP_dynamics


size_font = 30
mpl.rcdefaults()
mpl.rcParams['lines.linewidth'] = 8
mpl.rcParams['lines.markersize'] = 6
mpl.rcParams['patch.linewidth'] = 2
mpl.rcParams['axes.grid'] = True
mpl.rcParams['axes.labelsize'] = size_font
mpl.rcParams['font.family'] = 'sans-serif'
mpl.rcParams['font.size'] = size_font
# mpl.rcParams['font.serif'] = ['Times New Roman', 'Times', 'Bitstream Vera Serif', 'DejaVu Serif',
#                               'New Century Schoolbook',
#                               'Century Schoolbook L', 'Utopia', 'ITC Bookman', 'Bookman', 'Nimbus Roman No9 L',
#                               'Palatino',
#                               'Charter', 'serif']
mpl.rcParams['text.usetex'] = True
mpl.rcParams['legend.fontsize'] = size_font
mpl.rcParams['legend.loc'] = 'best'
mpl.rcParams['figure.facecolor'] = 'white'
# mpl.rcParams['figure.figsize'] = 14, 14
mpl.rcParams['savefig.format'] = 'pdf'

plt.ion()

L = 0.25
T_start = [0.1, 0.18, 0.26, 0.34, 0.42]
N = len(T_start)
# T_start = [0.2, 0.4, 0.6, 0.8]
step= 0.002
t_max = 1.3
samples = int(t_max / step)+1

com_ref_log = [np.full((3, samples), np.nan),
               np.full((3, samples), np.nan),
               np.full((3, samples), np.nan),
               np.full((3, samples), np.nan),
               np.full((3, samples), np.nan)]

t_log = [np.full(samples, np.nan),
         np.full(samples, np.nan),
         np.full(samples, np.nan),
         np.full(samples, np.nan),
         np.full(samples, np.nan)]

for i in range(N):
    slip_dyn = SLIP_dynamics(  0.002,
                               L,
                               0.5*L,
                               12.0,
                               9.81,
                               1., 1., 0.,
                               1.3)
    init_pos = np.array([0., 0., L])
    init_vel = np.array([1., 0., -9.81*T_start[i]])
    slip_dyn.def_and_solveOCP(init_pos, init_vel)
    slip_dyn.xy_dynamics()
    com_ref_log[i][:, :slip_dyn.ctrl_horz-1] =  slip_dyn.T_p_com_ref[:, :slip_dyn.ctrl_horz-1]
    t_log[i][:slip_dyn.ctrl_horz - 1] = slip_dyn.ctrl_time[:slip_dyn.ctrl_horz - 1]



# pad or clean
for i in range(5):
    ctrl = np.nanargmax(t_log[i])
    t_max = t_log[i][ctrl]
    if t_max <= 1.2:
        com_ref_log[i][:, ctrl:] = com_ref_log[i][:, ctrl].reshape(3,1)
    com_ref_log[i][:, int((1.2-T_start[i])/step+1):] = np.nan


t_plot = np.linspace(0, 1.3, samples)

fig, ax = plt.subplots(2, 1)
for i, t in enumerate(T_start):
    ax[0].plot(t+t_plot, com_ref_log[i][0,:])
    ax[1].plot(t+t_plot, com_ref_log[i][2,:])

ax[0].set_xlim(-0.05, 1.35)
ax[0].set_ylim(-0.01, 0.15)
ax[1].set_xlim(-0.05, 1.35)
ax[1].set_ylim(-0.01, 0.3)


ax[0].set_xticks([0.1, 0.3, 0.5, 0.7, 0.7, 0.9, 1.1, 1.3])
ax[1].set_xticks([0.1, 0.3, 0.5, 0.7, 0.7, 0.9, 1.1, 1.3])

ax[0].set_xticklabels([])
ax[1].set_xticklabels([])

ax[0].set_yticklabels(['','$0$'])
ax[1].set_yticklabels(['','$0$'])

ax[1].set_xlabel("$t \ [s]$")

ax[0].set_ylabel("$c^x(t) \ [m]$")
ax[1].set_ylabel("$c^z(t) \ [m]$")

ax[1].hlines(0.5*L, ax[1].get_xlim()[0], ax[1].get_xlim()[1], linestyles='dashed')

