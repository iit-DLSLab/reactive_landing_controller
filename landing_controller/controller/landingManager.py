import numpy as np
from .landingController import LandingController
from .utility import makePlots, saveInitConds
import copy
import datetime
import os
import pinocchio as pin
import rospy as ros

class LandingManager:
    def __init__(self, p, settings = None, noise=None):
        self.p = p
        self.lc = None

        self.settings = settings
        self.noise = noise

    def run(self, simulation_counter, basePose_init=None, baseTwist_init=None, useIK=False, useWBC=True, typeWBC='projection', naive=False):
        #########
        # reset #
        #########
        if self.p.real_robot:
            q_des = self.p.q.copy()
            qd_des = np.zeros_like(q_des)
            tau_ffwd = np.zeros_like(q_des)

            self.lc = LandingController(robot=self.p.robot,
                                        dt=2 * self.p.dt,
                                        q0=np.hstack([self.p.u.linPart(self.p.basePoseW),
                                                      self.p.quaternion,
                                                      q_des]),
                                        smoothing_param = 0.02)

        else:
            q_des = self.p.qj_0.copy()
            qd_des = np.zeros_like(q_des)
            tau_ffwd = np.zeros_like(q_des)
            self.p.unpause_physics_client()
            self.p.reset(basePoseW=basePose_init,
                         baseTwistW=baseTwist_init,
                         resetPid=useWBC)  # if useWBC = True, gains of pid are modified in landing phase
            baseLinVelW_init = baseTwist_init[:3].copy()
            if self.noise is not None:
                if 'horz_vel_init' in self.noise:
                    baseLinVelW_init[:2] += self.noise['horz_vel_init'].draw()
                print("Noisy initial horizontal velocity: " + str(baseLinVelW_init[:2]) + " [m/s]", flush=True)


            q_des = self.p.q.copy()
            self.lc = LandingController(robot=self.p.robot,
                                        dt=2 * self.p.dt,
                                        q0=np.hstack([self.p.u.linPart(self.p.basePoseW),
                                                      self.p.quaternion,
                                                      q_des]),
                                        smoothing_param = 0.02,
                                        naive = naive)

            self.lc.setCheckTimings(expected_lift_off_time=None,
                                    expected_apex_time=None,
                                    expected_touch_down_time=self.p.time + np.sqrt(2*(self.p.basePoseW[2]-0.25)/self.lc.g_mag) ,
                                    clearance=0.05) # about 12 mm of error in touch down
            if self.settings['VIDEO']['save']:
                # check if some old jpg are still in /tmp
                remove_jpg_cmd = "rm /tmp/camera_save/*"
                os.system(remove_jpg_cmd)

            self.p.setGravity(-self.lc.g_mag)

        ###############################
        #### FINITE STATE MACHINE #####
        ###############################
        # STATES
        # fsm_state = 0 - before APEX: kinematic adjustment (not used in simulations)
        # fsm_state = 1 - from APEX to TOUCH DOWN: compute landing trajectory + kinematic adjustment
        # fsm_state = 2 - from TOUCH DOWN to END: use last landing trajectory
        # fsm_state = 3 - quit
        fsm_state = 0
        # TRANSITIONS
        # iff isApexReached == True, fsm_state: 0 -> 1
        # iff isTouchDownOccurred == True, fsm_state: 1 -> 2


        start_time = self.p.time
        flag5s = False
        base_collided = False
        kfes_collided = False
        while not ros.is_shutdown():
            # print('fsm_state:', fsm_state, 'isApexReached:', isApexReached, 'isTouchDownOccurred:', isTouchDownOccurred)
            # update kinematic and dynamic model
            self.p.updateKinematics(update_legOdom=self.lc.lc_events.touch_down.detected, noise=self.noise)
            # check for collisions (only in simualtion)
            if not self.p.real_robot:
                base_collided = base_collided or self.p.checkBaseCollisions()
                kfes_collided = kfes_collided or self.p.checkKFECollisions()

            # self.p.visualizeContacts()

            ###############################################
            # STATE 0 - before APEX: kinematic adjustment #
            ###############################################
            if fsm_state == 0:
                # check if apex is reached
                if self.p.real_robot:
                    if self.p.time - start_time > 5:
                        if flag5s == False:
                            print("You can drop the robot now", flush=True)
                            self.p.imu_utils.baseLinTwistImuW[:] = 0.
                            flag5s = True
                        self.lc.apexReachedReal(t=self.p.time, sample=self.p.log_counter,
                                                baseLinAccW=self.p.baseLinAccW, window=1,
                                                threshold=-3)
                else:
                    self.lc.apexReached(t=self.p.time,
                                        sample=self.p.log_counter,
                                        vel_z_pre=self.p.comTwistW_log[2, self.p.log_counter - 1],
                                        vel_z_now=self.p.comTwistW[2])

                if self.lc.lc_events.apex.detected:
                    fsm_state += 1
                    for elem in self.p.contact_state:
                        elem = False  # to be sure that contact state is false when flying down
                else:
                    # # kinematic adjustment
                    # self.lc.flyingUp_phase(self.p.b_R_w)
                    # # set references
                    # for i, leg in enumerate(self.lc.legs): # ['lf', 'rf', 'lh', 'lh']
                    #     q_des_leg, isFeasible  = self.p.IK.ik_leg(self.lc.B_feet_task[i],
                    #                                          self.p.robot.model.getFrameId(leg+'_foot'),
                    #                                          self.p.legConfig[leg][0],
                    #                                          self.p.legConfig[leg][1])
                    #     if isFeasible:
                    #         self.p.u.setLegJointState(i, q_des_leg, q_des)
                    # qd_des = np.zeros(self.p.robot.na)
                    # tau_ffwd = self.p.self_weightCompensation()
                    pass

            ########################################################################################
            # STATE 1 - from APEX to TOUCH DOWN: compute landing trajectory + kinematic adjustment #
            ########################################################################################

            if fsm_state == 1:
                # check if touch down is occurred
                if self.p.real_robot:
                    self.lc.touchDownReal(t=self.p.time,
                                          sample=self.p.log_counter,
                                          contacts_state=self.p.contact_state)
                else:
                    self.lc.touchDown(t=self.p.time,
                                      sample=self.p.log_counter,
                                      contacts_state=self.p.contact_state)

                if self.lc.lc_events.touch_down.detected:
                    fsm_state += 1
                    height = 0.
                    for leg in range(4):
                        height -= self.p.B_contacts[leg][2]
                    height /= 4
                    self.p.leg_odom.reset(np.hstack([0., 0., height, self.p.quaternion, self.p.q]))
                    # if use only ik -> same pid gains
                    # if use ik + wbc -> reduce pid gains
                    # if only wbc -> zero pid gains
                    if not useIK:
                        self.p.pid.setPDs(0., 0., 0.)
                    elif useWBC and useIK:
                        self.p.pid.setPDjoints(self.p.kp_wbc_j, self.p.kd_wbc_j, self.p.ki_wbc_j)
                    # elif not simulation['useWBC'] and simulation['useIK'] :
                    #       do not change gains

                    # save the  position at touch down
                    comPoseH, comRateH = self.p.World2Hframe(self.p.comPoseW, self.p.comTwistW)
                    self.lc.landed(comPoseH, comRateH)

                    self.p.W_contacts_TD = copy.deepcopy(self.p.W_contacts)

                else:
                    # compute landing trajectory + kinematic adjustment
                    w_R_hf = pin.rpy.rpyToMatrix(0, 0, self.p.u.angPart(self.p.basePoseW)[2])

                    if self.p.real_robot:
                        self.lc.flyingDown_phase(self.p.b_R_w @ w_R_hf, w_R_hf.T @ self.p.u.linPart(self.p.baseTwistW))
                    else:
                        baseLinVelW_init[2] = self.p.baseTwistW[2]
                        self.lc.flyingDown_phase(self.p.b_R_w @ w_R_hf, w_R_hf.T @ baseLinVelW_init)

                    # set references
                    # joints position
                    for i, leg in enumerate(self.lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                        q_des_leg, isFeasible = self.p.IK.ik_leg(self.lc.B_feet_task[i],
                                                                 leg,
                                                                 self.p.legConfig[leg][0],
                                                                 self.p.legConfig[leg][1], self.settings['verbose'])

                        if isFeasible:
                            self.p.u.setLegJointState(i, q_des_leg, q_des)

                        # joints velocity
                        B_contact_err = self.lc.B_feet_task[i] - self.p.B_contacts[i]
                        kv = .01 / self.p.dt
                        self.p.B_vel_contacts_des[i] = kv * B_contact_err
                        qd_des_leg = self.p.J_inv[i] @ self.p.B_vel_contacts_des[i]
                        self.p.u.setLegJointState(i, qd_des_leg, qd_des)

                    tau_ffwd = self.p.self_weightCompensation()

            #################################################################
            # STATE 2 - from TOUCH DOWN to END: use last landing trajectory #
            #################################################################
            if fsm_state == 2:
                if self.lc.lp_counter > 2 * self.lc.ref_k + 100:  # take a bit before quitting
                    fsm_state = 3
                    break
                else:
                    # use last computed trajectories
                    self.lc.landed_phase(self.p.time)

                    # set references
                    b_R_w_des = pin.rpy.rpyToMatrix(self.p.u.angPart(self.lc.pose_des)).T
                    omega_des_skew = pin.skew(b_R_w_des @ self.p.u.angPart(self.lc.pose_des))
                    # joints position
                    for i, leg in enumerate(self.lc.legs):  # ['lf', 'lh', 'rf','rh']
                        q_des_leg, isFeasible = self.p.IK.ik_leg(self.lc.B_feet_task[i],
                                                                 leg,
                                                                 self.p.legConfig[leg][0],
                                                                 self.p.legConfig[leg][1],
                                                                 self.settings['verbose'])
                        if isFeasible:
                            self.p.u.setLegJointState(i, q_des_leg, q_des)

                        # joints velocity
                        self.p.B_vel_contacts_des[i] = omega_des_skew.T @ self.lc.B_feet_task[
                            i] - b_R_w_des @ self.p.u.linPart(self.lc.twist_des)
                        qd_leg_des = self.p.J_inv[i] @ self.p.B_vel_contacts_des[i]
                        self.p.u.setLegJointState(i, qd_leg_des, qd_des)

                    if not useWBC:
                        tau_ffwd = self.p.gravityCompensation()

                    # save LC references for com, feet, zmp
                    self.p.comPoseW_des, self.p.comTwistW_des, comAccW_des = self.p.Hframe2World(self.lc.pose_des,
                                                                                                 self.lc.twist_des,
                                                                                                 self.lc.acc_des)

                    if useWBC:
                        tau_ffwd = self.p.WBC(self.p.comPoseW_des, self.p.comTwistW_des, comAccW_des, type=typeWBC)



            for leg in range(4):
                self.p.W_contacts_des[leg] = self.p.u.linPart(self.p.basePoseW) + self.p.b_R_w.T @ self.lc.B_feet_task[leg]
                self.p.B_contacts_des[leg] = self.lc.B_feet_task[leg].copy()

            # finally, send commands
            self.p.send_command(q_des, qd_des, tau_ffwd)

        if not self.p.real_robot:
            self.p.pause_physics_client()
            # self.p.q_des = self.p.qj_0.copy()
            # self.p.qd_des = np.zeros(self.p.robot.na)
            # self.p.tau_ffwd = np.zeros(self.p.robot.na)


            self.settings['SIMS'][simulation_counter]['directory'] = self.settings['save_path']+'/sim' + self.settings['SIMS'][simulation_counter]['id']
            if self.settings['save_path']:
                os.mkdir(self.settings['SIMS'][simulation_counter]['directory'])
                saveInitConds(self.settings['SIMS'][simulation_counter]['directory'],
                              self.settings['SIMS'][simulation_counter],
                              speedUpDown= self.settings['VIDEO']['speedUpDown'],
                              verbose=self.settings['verbose'])

            if self.settings['PLOTS']['save'] or self.settings['PLOTS']['show']:
                makePlots(p=self.p, figures=self.settings['FIGURES'], PLOT_SETTINGS=self.settings['PLOTS'],
                          directory=self.settings['SIMS'][simulation_counter]['directory'],
                          start = 0, end = -1, verbose = self.settings['verbose'])

            # save all for later analysis
            if self.settings['WORKSPACE']['save']:
                EXTRADATA = {}
                EXTRADATA["lift_off_sample"] = self.lc.lc_events.lift_off.sample
                EXTRADATA["lift_off_t"] = self.lc.lc_events.lift_off.t
                EXTRADATA["apex_sample"] = self.lc.lc_events.apex.sample
                EXTRADATA["apex_t"] = self.lc.lc_events.apex.t
                EXTRADATA["touch_down_sample"] = self.lc.lc_events.touch_down.sample
                EXTRADATA["touch_down_t"] = self.lc.lc_events.touch_down.t
                self.p.saveData(self.settings['SIMS'][simulation_counter]['directory'], EXTRADATA=EXTRADATA,
                                start=self.lc.lc_events.apex.sample, stop=self.p.log_counter,
                                verbose = self.settings['verbose'])

            if self.settings['VIDEO']['save']:
                # save the video
                self.p.saveVideo(self.settings['SIMS'][simulation_counter]['directory'],
                                  speedUpDown=self.settings['VIDEO']['speedUpDown'])

            feet_in_touch = True
            for leg in range(4):
                feet_in_touch = feet_in_touch and self.p.W_contacts[leg][2]<0.03


            convergence = np.linalg.norm( self.p.q - self.p.qj_0 ) < 1.

            stable_standing = np.linalg.norm(self.p.qd) < 0.6

            # at the end I want no collision and feet on ground!
            print("    base not collided?", not base_collided, flush=True)
            print("    kfes not collided?", not kfes_collided, flush=True)
            print("    feet in touch?", feet_in_touch, flush=True)
            print("    convergence?", convergence, ": norm( q - q0 )=" + str(np.linalg.norm( self.p.q - self.p.qj_0 ))+" [rad]", flush=True)
            print("    stable standing?", stable_standing, ": norm(qd)=" + str(np.linalg.norm(self.p.qd))+" [rad/s]", flush=True)
            ret = (not base_collided) and (not kfes_collided) and feet_in_touch and convergence and stable_standing
        else:
            ret = True # the collision and feet on ground checks are not relevant on real robot
        return ret


