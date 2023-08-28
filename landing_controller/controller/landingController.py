import numpy as np
import pinocchio as pin
from base_controllers.utils.utils import Utils
from landing_controller.lib.SLIP_dynamics_lib import SLIP_dynamics
from landing_controller.controller.utility import LcEvents

from matplotlib import pyplot as plt

# In the class appears vectors and matrices describing pose and twist. Explanation:
# A_p     : position of a generic point expressed wrt frame A
# A_o_B   : position of the origin of frame B expressed wrt frame A
# A_R_B   : rotation of frame B wrt frame A
# A_v     : linear velocity of a generic point expressed wrt frame A
# A_v_B   : linear velocity of the origin of frame B wrt frame A
# A_omega : angular velocity of frame A

# The variables defining the state along a certain axis respect the following convention:
# stateX = [velocityX]
#          [positionX]

class LandingController:
    def __init__(self, robot, dt, q0, g_mag=9.81, smoothing_param = 0.005, naive=False):
        self.u = Utils()

        self.naive = naive
        self.robot = robot
        self.qj_home = q0[7:]
        self.v_home = pin.utils.zero(self.robot.nv)

        self.dt = dt

        self.ref_k = -1
        self.lp_counter = -1

        # checking times: setCheckTimings() should be called before run
        self.check_lift_off_time = 0.
        self.check_apex_time = 0.
        self.check_touch_down_time = 0.

        # MSD params
        self.g_mag = g_mag
        self.m = self.robot.robot_mass

        # these variables are useful for computations
        # Neutral: base expressed in base frame
        # q_neutral[0:7] and v_neutral[0:6] cannot be modified
        self._q_neutral = pin.neutral(self.robot.model)

        self.foot2base = self.base_height(q_j=self.qj_home)
        # here I assume all the feet has the same radius
        floor2foot = 0.
        for obj in robot.collision_model.geometryObjects:
            if "foot" in obj.name:
                if hasattr(obj.geometry, 'radius'):
                    floor2foot = obj.geometry.radius
            break

        self._q_neutral[2] = self.foot2base + floor2foot
        self.com_home = self.robot.robotComW(self._q_neutral)
        self.L = self.com_home[2]

        self.max_spring_compression = 0.4 * self.L
        w_v = 1.
        w_p = 1.
        w_u = 0.
        max_settling_time = 1.8

        self.slip_dyn = SLIP_dynamics(  self.dt,
                                        self.L,
                                        self.max_spring_compression,
                                        self.m,
                                        self.g_mag,
                                        w_v, w_p, w_u,
                                        max_settling_time)

        self.euler_TD = np.zeros(3)
        self.eig_ang = 0.

        self.pose_des = np.zeros(6)
        self.twist_des = np.zeros(6)
        self.acc_des = np.zeros(6)

        self.T_comPose_TD = np.zeros(6)
        self.T_comTwist_TD = np.zeros(6)

        self.T_o_B = np.array([0, 0, self.L])

        self.B_feet_home = []
        self.T_feet_home = []

        self.B_feet_task = []
        self.T_feet_task = []

        self.legs = ['lf', 'lh', 'rf', 'rh']

        for leg in self.legs:
            name = leg+'_foot'
            B_foot = self.robot.data.oMf[self.robot.model.getFrameId(name)].translation
            self.B_feet_home.append(B_foot.copy())
            self.B_feet_task.append(B_foot.copy())

            T_foot = np.array([B_foot[0], B_foot[1], 0.0])
            self.T_feet_home.append(T_foot.copy())
            self.T_feet_task.append(T_foot.copy())


        self.init_pos = np.zeros([3, 1])
        self.init_vel = np.zeros([3, 1])

        self.alpha = 0.0
        self.smoothing_param = smoothing_param

        self.euler_final = np.zeros(3)


        self.lc_events = LcEvents()


    def base_height(self, q_j):
        # assumes all the feet in contact and contact surface horizontal
        self._q_neutral[7:] = q_j
        pin.forwardKinematics(self.robot.model, self.robot.data, self._q_neutral)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        foot2base = 0.
        for ee_id in self.robot.getEndEffectorsFrameId:
            foot2base -= self.robot.data.oMf[ee_id].translation[2].copy()
        return foot2base/4


    def pushing_phase(self):
        pass

    def flyingUp_phase(self, B_R_T):
        # kinematic adjustment: drive feet to create a plane parallel to landing surface and located at distance
        # self.L from the base
        # i.e., move feet to the home position according to the rotation of the base

        # at the beginning, B-frame and T-frame have the same orientation, thus at this stage we only have to rotate the
        # feet position expressed in base frame
        for leg in range(4):
            self.B_feet_task[leg] = B_R_T @ (self.T_feet_task[leg] - self.T_o_B)


    def flyingDown_phase(self, B_R_T, com_vel_T):
        # (re-)compute zmp and move feet plane accordingly
        # self.init_pos[0] = 0.
        # self.init_pos[1] = 0.
        # self.init_pos[2] = self.L
        self.init_vel[0] = com_vel_T[0]
        self.init_vel[1] = com_vel_T[1]
        self.init_vel[2] = com_vel_T[2]

        self.slip_dyn.def_and_solveOCP(self.init_pos, self.init_vel)

        # task for feet in terrain frame
        if self.alpha < 1.:
            self.alpha = np.around(self.alpha+self.smoothing_param, 3)

        if self.naive:
            self.slip_dyn.zmp_xy[0] = 0.0
            self.slip_dyn.zmp_xy[1] = 0.0

        for leg in range(4):
            self.T_feet_task[leg][0] = self.T_feet_home[leg][0] + self.alpha * self.slip_dyn.zmp_xy[0]
            self.T_feet_task[leg][1] = self.T_feet_home[leg][1] + self.alpha * self.slip_dyn.zmp_xy[1]
            self.B_feet_task[leg] = B_R_T @ (self.T_feet_task[leg] - self.T_o_B)


    def landed(self, T_comPose, T_comTwist):
        self.T_comPose_TD = T_comPose.copy()
        self.T_comTwist_TD = T_comTwist.copy()
        self.slip_dyn.xy_dynamics()
        self.eig_ang = -np.sqrt(self.slip_dyn.K / self.slip_dyn.m)

        self.pose_des[3:5] = self.T_comPose_TD[3:5]
        self.pose_des[5] = 0.
        self.twist_des[3:] = self.T_comTwist_TD[3:]

        self.MAT_ANG = np.eye(2) + np.array([[- self.slip_dyn.D / self.slip_dyn.m,  -self.slip_dyn.K / self.slip_dyn.m], [1, 0]]) * self.dt

        self.Rdyn = np.vstack([self.twist_des[3], self.pose_des[3]])
        self.Pdyn = np.vstack([self.twist_des[4], self.pose_des[4]])
        self.Ydyn = np.vstack([self.twist_des[5], self.pose_des[5]])


    def landed_phase(self, t):
        # use the last trajectory computed
        # task for feet in terrain frame

        # lp_counter counts how many times landed_phase has been called. the reference computed by slip_dyn.xy_dynamics()
        # we apply twice the reference. so ref_k is half of lp_counter, up to its max value
        self.lp_counter += 1
        if self.ref_k < self.slip_dyn.ctrl_horz-1:
            if self.lp_counter %2 == 0:
                self.ref_k += 1


        # REFERENCES
        if self.naive: # naive controller (not use this in real robot)
            self.Rdyn = self.MAT_ANG @ self.Rdyn
            self.Pdyn = self.MAT_ANG @ self.Pdyn
            self.Ydyn = self.MAT_ANG @ self.Ydyn
            # ---> POSE
            # to avoid reshape, use these three lines
            self.pose_des[0] = self.T_comPose_TD[0]
            self.pose_des[1] = self.T_comPose_TD[1]
            self.pose_des[2] = self.slip_dyn.T_p_com_ref[2, self.ref_k]

            self.pose_des[3] = self.Rdyn[1]
            self.pose_des[4] = self.Pdyn[1]
            self.pose_des[5] = self.Ydyn[1] + self.T_comPose_TD[5]

            B_R_T_des = pin.rpy.rpyToMatrix(self.pose_des[3], self.pose_des[4], 0).T

            # ---> TWIST
            self.twist_des[0] = 0
            self.twist_des[1] = 0
            self.twist_des[2] = self.slip_dyn.T_v_com_ref[2, self.ref_k]

            self.twist_des[3] = self.Rdyn[0]
            self.twist_des[4] = self.Pdyn[0]
            self.twist_des[5] = self.Ydyn[0]

            # ---> ACCELERATION
            self.acc_des[0] = 0
            self.acc_des[1] = 0
            self.acc_des[2] = self.slip_dyn.T_a_com_ref[2, self.ref_k]

            Dm = self.slip_dyn.D / self.slip_dyn.m
            Km = self.slip_dyn.K / self.slip_dyn.m
            self.acc_des[3] = -Dm * self.Rdyn[0] - Km * self.Rdyn[1]
            self.acc_des[4] = -Dm * self.Pdyn[0] - Km * self.Pdyn[1]
            self.acc_des[5] = -Dm * self.Ydyn[0] - Km * self.Ydyn[1]

            self.T_o_B[2] = self.pose_des[2]

            for leg in range(4):
                self.T_feet_task[leg][0] = self.T_feet_home[leg][0] + self.alpha * self.slip_dyn.zmp_xy[0]
                self.T_feet_task[leg][1] = self.T_feet_home[leg][1] + self.alpha * self.slip_dyn.zmp_xy[1]
                self.B_feet_task[leg] = B_R_T_des @ (self.T_feet_task[leg] - self.pose_des[:3])

        else: # lc controller
            self.Rdyn = self.MAT_ANG @ self.Rdyn
            self.Pdyn = self.MAT_ANG @ self.Pdyn
            self.Ydyn = self.MAT_ANG @ self.Ydyn
            # ---> POSE
            # to avoid reshape, use these three lines
            self.pose_des[0] = self.T_comPose_TD[0] + self.slip_dyn.T_p_com_ref[0, self.ref_k]
            self.pose_des[1] = self.T_comPose_TD[1] + self.slip_dyn.T_p_com_ref[1, self.ref_k]
            self.pose_des[2] = self.slip_dyn.T_p_com_ref[2, self.ref_k]

            self.pose_des[3] = self.Rdyn[1]
            self.pose_des[4] = self.Pdyn[1]
            self.pose_des[5] = self.Ydyn[1] + self.T_comPose_TD[5]

            B_R_T_des = pin.rpy.rpyToMatrix(self.pose_des[3], self.pose_des[4], 0).T

            # ---> TWIST
            self.twist_des[0] = self.slip_dyn.T_v_com_ref[0, self.ref_k]
            self.twist_des[1] = self.slip_dyn.T_v_com_ref[1, self.ref_k]
            self.twist_des[2] = self.slip_dyn.T_v_com_ref[2, self.ref_k]

            self.twist_des[3] = self.Rdyn[0]
            self.twist_des[4] = self.Pdyn[0]
            self.twist_des[5] = self.Ydyn[0]

            # ---> ACCELERATION
            self.acc_des[0] = self.slip_dyn.T_a_com_ref[0, self.ref_k]
            self.acc_des[1] = self.slip_dyn.T_a_com_ref[1, self.ref_k]
            self.acc_des[2] = self.slip_dyn.T_a_com_ref[2, self.ref_k]

            Dm = self.slip_dyn.D / self.slip_dyn.m
            Km = self.slip_dyn.K / self.slip_dyn.m
            self.acc_des[3] = -Dm * self.Rdyn[0] - Km * self.Rdyn[1]
            self.acc_des[4] = -Dm * self.Pdyn[0] - Km * self.Pdyn[1]
            self.acc_des[5] = -Dm * self.Ydyn[0] - Km * self.Ydyn[1]

            self.T_o_B[2] = self.pose_des[2]

            for leg in range(4):
                self.T_feet_task[leg][0] = self.T_feet_home[leg][0] + self.alpha * self.slip_dyn.zmp_xy[0]
                self.T_feet_task[leg][1] = self.T_feet_home[leg][1] + self.alpha * self.slip_dyn.zmp_xy[1]
                self.B_feet_task[leg] = B_R_T_des @ (
                            self.T_feet_task[leg] - self.slip_dyn.T_p_com_ref[:, self.ref_k])


    def setCheckTimings(self, expected_lift_off_time=None, expected_apex_time=None, expected_touch_down_time=None, clearance=None):
        if clearance is None:
            clearance = 0.

        if expected_lift_off_time is not None:
            self.check_lift_off_time = expected_lift_off_time - clearance
        else:
            self.check_lift_off_time = - clearance

        if expected_apex_time is not None:
            self.check_apex_time = expected_apex_time - clearance
        else:
            self.check_apex_time = self.check_lift_off_time

        if expected_touch_down_time is not None:
            self.check_touch_down_time = expected_touch_down_time - clearance
        else:
            self.check_touch_down_time = self.check_apex_time


        assert self.check_lift_off_time <= self.check_apex_time, 'expected_lift_off_time should be >= expected_apex_time'
        assert self.check_apex_time <= self.check_touch_down_time, 'expected_touch_down_time should be >= expected_apex_time'

    def liftOff(self, t, sample, contact_state):
        anyLO = False
        if t > self.check_lift_off_time:
            anyLO = not any(contact_state)
            if anyLO:
                self.lc_events.lift_off.set(t, sample)
        return anyLO

    def apexReachedReal(self, t, sample, baseLinAccW, window=1, threshold=-1.5):
        if baseLinAccW[2]<threshold:
            self.lc_events.apex.set(t, sample)
            return True
        else:
            return False


    def apexReached(self, t, sample, vel_z_pre, vel_z_now):
        if t > self.check_apex_time:
            if (vel_z_pre >=0. and vel_z_now < 0.) or (vel_z_pre <0. and vel_z_now < 0.):
                self.lc_events.apex.set(t, sample)
                return True

        return False

    def touchDownReal(self, t, sample, contacts_state):
        allTD = all(contacts_state)
        if allTD:
            self.lc_events.touch_down.set(t, sample)

        return allTD

    def touchDown(self, t, sample, contacts_state):
        if t >= self.check_touch_down_time:
            allTD = all(contacts_state)
            if allTD:
                self.lc_events.touch_down.set(t, sample)
            return allTD
        else:
            return False

    def velocity_margin(self, sp):
        limit_zmp = self.marginal_zmp(sp)
        P_x_last = self.slip_dyn.P_xy_stack[-2:, -2:]
        P_u_last = self.slip_dyn.P_u_stack[-2:, ].reshape(2, 1)

        # H = np.array([[0], [1]]) - P_u_last
        # G = np.linalg.inv(P_x_last)
        H = np.vstack([np.array([[0], [1]]) - P_u_last, 0])
        G = np.vstack([P_x_last, np.array([0, 1])])

        L = np.linalg.pinv(G) @ H

        limit_vx = (L * limit_zmp[0])[0]
        limit_vy = (L * limit_zmp[1])[0]

        marg_vx = limit_vx - self.init_vel[0]
        marg_vy = limit_vy - self.init_vel[1]

        eta = L[0]

        return eta, limit_zmp, marg_vx, marg_vy, limit_vx, limit_vy


    def marginal_zmp(self, sp):
        # marginal zmp is the intersection between the line defined by the touch down velocity and the support polygon
        # in the direction of the touch down velocity
        # sp lines equation : y = m*x+q
        # v td line equation: y = r*x
        # Wcontacts: lf, rf, lh, rh

        r = self.init_vel[1]/self.init_vel[0]
        angle_vTD = np.arctan2(self.init_vel[1], self.init_vel[0])
        m_zmp = np.zeros(2)
        for side in sp:
            q = sp[side]['q']
            m = sp[side]['m']
            zmp_x = q/(r-m)
            zmp_y = r * zmp_x
            # check if candidate zmp is in the bounds of the line
            if sp[side]['p0'][0] < zmp_x < sp[side]['p1'][0] and sp[side]['p0'][1] < zmp_y < sp[side]['p1'][1]:
                # check if the zmp is in the correct direction
                angle_zmp = np.arctan2(zmp_y, zmp_x)
                if np.abs(angle_vTD - angle_zmp)< 1e-5:
                    m_zmp[0] = zmp_x
                    m_zmp[1] = zmp_y
                    break
        return m_zmp


    def plot_margins(self, sp):
        fig, ax = plt.subplots()
        X_vals = []
        Y_vals = []
        for side in sp:
            X_vals.append(sp[side]['p0'][0])
            Y_vals.append(sp[side]['p0'][1])
        X_vals.append(X_vals[0])
        Y_vals.append(Y_vals[0])
        plt.plot(X_vals, Y_vals)

        self.m_zmp = self.marginal_zmp(sp)
        ax.add_patch(plt.Circle((self.slip_dyn.zmp_xy[0], self.slip_dyn.zmp_xy[1]), 0.02, color='r'))
        ax.add_patch(plt.Circle((self.m_zmp[0], self.m_zmp[1]), 0.02, color='b'))

        plt.plot([0, self.init_vel[0]], [0, self.init_vel[1]])

        ax.set_aspect('equal', adjustable='box')

        plt.show()

    def plot_ref(self, figure_id=None):
        label = 'CoM '
        axes = ['X', 'Y', 'Z']
        if figure_id is None:
            figure_id = 1000
        fig = plt.figure(figure_id)
        ax = fig.subplots(3, 2)

        ax[2, 0].axhline(y=self.slip_dyn.L - self.slip_dyn.max_spring_compression, color='r', linestyle='--')
        for i in range(0, 3):
            ax[i, 0].plot(self.slip_dyn.ctrl_time[:self.slip_dyn.ctrl_horz],
                         self.slip_dyn.T_p_com_ref[i, :self.slip_dyn.ctrl_horz].T, color='r', linewidth=4.)
            ax[i, 0].set_ylabel(label + axes[i] + '[m]')
            ax[i, 0].grid()
            ax[i, 0].set_xlabel('t [s]')

        for i in range(0, 3):
            ax[i, 1].plot(self.slip_dyn.ctrl_time[:self.slip_dyn.ctrl_horz],
                     self.slip_dyn.T_v_com_ref[i, :self.slip_dyn.ctrl_horz].T, color='r', linewidth=4.)
            ax[i, 1].set_ylabel('V '+ label + axes[i] + '[m/s]')
            ax[i, 0].set_xlabel('t [s]')
            ax[i, 1].grid()
            ax[i, 1].axhline(y=0., color='r', linestyle='--')
            ax[i, 0].set_xlabel('t [s]')
