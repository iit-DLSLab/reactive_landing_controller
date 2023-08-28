from landing_controller.lib.SLIP_dynamics_lib import SLIP_dynamics
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import time

dt = 0.004
L = 0.259
max_spring_compression = 1/2 * L
m = 12.0
g_mag = 9.81
w_v = 1.
w_p = 1.
w_u = 1e-6
max_settling_time = 1.5

s = SLIP_dynamics(dt, L, max_spring_compression, m, g_mag, w_v, w_p, w_u, max_settling_time)


fig_p, ax_p = plt.subplots(3,1)
fig_v, ax_v = plt.subplots(3,1)
fig_a, ax_a = plt.subplots(3,1)

fig_omega, ax_omega = plt.subplots()

fig_pz, ax_pz = plt.subplots()

fig_zmp, ax_zmp = plt.subplots()

legend = []

zmp_x_list = []
zmp_y_list = []

zmp_x_ocp_list = []
zmp_y_ocp_list = []
zmp_colour_list = []
#v0_list = np.arange(1.,2.1, 0.2)

vx0_list =  [0.1, 1.]
# vz0_list =  [-5., -1, -0.1]
vz0_list =  [-0.2, -2.]


lx = len(vx0_list)
lz = len(vz0_list)
used_colors = []
for i in range(lx):
	color_usable = False
	if len(used_colors) != 0:
		while not color_usable:
			color = np.random.random(3)
			for us_col in used_colors:
				us_col_dir = us_col/np.linalg.norm(us_col)
				col_dir = color/np.linalg.norm(color)

				if np.linalg.norm(us_col_dir-col_dir)<0.5 or np.linalg.norm(color)<0.8: # I want different colors that are dark enough
					color_usable = False
					break
				else:
					color_usable = True
			if color_usable:
				used_colors.append(color)
				break

	else:
		color = np.random.random(3)
		used_colors.append(color)

i = -1
for vx in vx0_list:
	i += 1
	j = -1
	for vz in vz0_list:
		j+=1
		alpha = (j + 1) / lz
		init_pos = np.array([0., 0., L])
		init_vel = np.array([vx,  0.,  vz])
		init_state_x = np.array([[init_vel[0]], [init_pos[0]]])
		init_state_y = np.array([[init_vel[1]], [init_pos[1]]])
		#s.runVerbose(init_pos, init_vel)
		s.def_and_solveOCPVerbose(init_pos, init_vel)
		s.def_and_solveOCP(init_pos, init_vel)
		zmp_x_ocp_list.append(s.zmp_xy[0])
		zmp_y_ocp_list.append(s.zmp_xy[1])
		s.xy_dynamics()
		ax_p[0].plot(s.ctrl_time[:s.ctrl_horz], s.T_p_com_ref[0, :s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)
		ax_p[1].plot(s.ctrl_time[:s.ctrl_horz], s.T_p_com_ref[1, :s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)
		ax_p[2].plot(s.ctrl_time[:s.ctrl_horz], s.T_p_com_ref[2, :s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)

		ax_pz.plot(s.ctrl_time[:s.ctrl_horz], s.T_p_com_ref[2, :s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)

		ax_v[0].plot(s.ctrl_time[:s.ctrl_horz], s.T_v_com_ref[0, :s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)
		ax_v[1].plot(s.ctrl_time[:s.ctrl_horz], s.T_v_com_ref[1, :s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)
		ax_v[2].plot(s.ctrl_time[:s.ctrl_horz], s.T_v_com_ref[2, :s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)

		ax_a[0].plot(s.ctrl_time[:s.ctrl_horz], s.T_a_com_ref[0, :s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)
		ax_a[1].plot(s.ctrl_time[:s.ctrl_horz], s.T_a_com_ref[1, :s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)
		ax_a[2].plot(s.ctrl_time[:s.ctrl_horz], s.T_a_com_ref[2, :s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)

		ax_omega.plot(s.ctrl_time[:s.ctrl_horz], s.omega_sq_ref[:s.ctrl_horz].T, color=used_colors[i].tolist(), alpha=alpha)

		ax_zmp.scatter(s.zmp_xy[0], s.zmp_xy[1], color=used_colors[i].tolist(), alpha=alpha)

		legend.append('V0 = ' +str(np.around(init_vel, 2)) + ' m/s' + '\nK = ' + str(str(np.around(s.K))) + ' N/m')
ax_p[2].axhline(y=s.L-s.max_spring_compression, color='r', linestyle='--')
ax_v[0].axhline(y=0., color='r', linestyle='--')
ax_p[0].set_ylabel('pos com x [m]')
ax_p[1].set_ylabel('pos com y [m]')
ax_p[2].set_ylabel('pos com z [m]')
ax_v[0].set_ylabel('vel com x [m/s]')
ax_v[1].set_ylabel('vel com y [m/s]')
ax_v[2].set_ylabel('vel com z [m/s]')
ax_a[0].set_ylabel('acc com x [m/s^2]')
ax_a[1].set_ylabel('acc com y [m/s^2]')
ax_a[2].set_ylabel('acc com z [m/s^2]')
ax_pz.set_ylabel('pos com z [m]')
ax_p[2].set_xlabel('t [s]')
ax_v[2].set_xlabel('t [s]')
ax_a[2].set_xlabel('t [s]')
ax_pz.set_xlabel('t [s]')
ax_pz.legend(legend)





ax_zmp.set_title('zmp')
ax_zmp.set_ylabel('zmp y [m]')
ax_zmp.set_xlabel('zmp x [m]')
ax_zmp.legend(legend)
plt.show()


