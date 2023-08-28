# this is a copy of wbc.py
import time

from base_controllers.quadruped_controller import Controller
import rospy as ros
import numpy as np

from base_controllers.utils.common_functions import *
from base_controllers.utils.math_tools import polynomialRef
import matplotlib.pyplot as plt
import os

import pinocchio as pin

import base_controllers.params as conf

import cProfile, pstats, io
from pstats import SortKey

robotName = "go1"


#################
# Configuration #
#################
test = {}
test['duration'] = 5000.
test['stabilize'] = 1.
test['usePid'] = 1.
test['amp']  = np.array([0.03, 0., 0.03, 0., 0.0, 0])
test['phase']  = np.array([0., 0., np.pi/2, 0., 0, 0.])
test['delta']  = np.array([0.0, 0., 0.03, 0., 0.0, 0.])
test['delta']  = test['amp'] * np.sin(test['phase'] )
test['freq'] = 0.3
test['typeWBC'] = 'projection'
# test['typeWBC'] = 'qp'
# if use qp, the following must be set
test['normals'] = [np.array([0.,0.,1.])]*4
test['friction_coeffs'] = [0.8]*4

test['profile'] = True


if __name__ == '__main__':
    p = Controller(robotName)
    if test['profile']:
        pr = cProfile.Profile()

    test['pulse'] = 2*np.pi*test['freq']
    test['pulse2'] = test['pulse']**2
    try:

        p.startController(world_name='slow.world', use_ground_truth_contacts=False, additional_args=['gui:=true', 'go0_conf:=standDown', 'pid_discrete_implementation:=true'])

        p.startupProcedure()  # overloaded method

        if test['typeWBC'] == 'qp':
            p.setWBCConstraints(test['normals'], test['friction_coeffs'])

        # Reset reference to actual value
        state = 0
        time_init = p.time.copy()
        p.x0 = p.comPoseW.copy()
        p.comAccW_des = np.zeros(6)

        ####

        print('start!')

        p.q_des = p.q.copy()
        p.qd_des = p.qd.copy()

        # Control loop
        while not ros.is_shutdown():
            #print(p.time, state)
            # update the kinematics
            p.updateKinematics()
            collided = p.checkGroundCollisions()
            # p.visualizeContacts()
            # Reference Generation
            if state == 0:
                if p.time - time_init < test['usePid']:
                    p.gravityCompensation()
                else:
                    state += 1
                    p.x0 = p.comPoseW.copy()
                    p.pid.setPDjoints(p.kp_wbc_j, p.kd_wbc_j, p.ki_wbc_j)
                    # p.pid.setPDjoints(np.zeros(12),np.zeros(12),np.zeros(12))
                    time_stabilize = p.time.copy()

                    p.W_contacts_des = p.W_contacts.copy()

                    print('starting wbc ' + str(p.time) + " [s]")
            elif state == 1:
                if p.time - time_stabilize < test['stabilize']:
                    if test['profile']:
                        pr.enable()
                    p.comPoseW_des = p.Hframe2World(p.x0)
                    p.comTwistW_des[:] = 0.
                    p.comAccW_des[:] = 0.

                    p.Wcom2Joints_des()

                    p.WBC(p.comPoseW_des,
                          p.comTwistW_des,
                          p.comAccW_des,
                          comControlled=True,
                          type=test['typeWBC'])
                    if test['profile']:
                        pr.disable()
                else:
                    time_sinusoid = p.time.copy()
                    print('starting sinusoid ' + str(p.time) + " [s]")
                    state += 1

            elif state == 2:
                if p.time - time_sinusoid < test['duration']-(1/4)*(1/test['freq']):
                    # com ref
                    if test['profile']:
                        pr.enable()
                    comPoseH_des   = p.x0 + test['amp'] * np.sin( test['pulse']*(p.time - time_sinusoid) + test['phase']) -  test['delta']
                    dcomPoseH_des  =  test['pulse'] * test['amp'] * np.cos( test['pulse']*(p.time - time_sinusoid) + test['phase'] )
                    ddcomPoseH_des = -test['pulse2'] * test['amp'] * np.sin( test['pulse']*(p.time - time_sinusoid) + test['phase'] )

                    p.comPoseW_des, p.comTwistW_des, p.comAccW_des = p.Hframe2World(comPoseH_des, dcomPoseH_des, ddcomPoseH_des)

                    p.Wcom2Joints_des()
                    
                    p.WBC(p.comPoseW_des,
                          p.comTwistW_des,
                          p.comAccW_des,
                          comControlled=True,
                          type=test['typeWBC'])
                    if test['profile']:
                        pr.disable()
                else:
                    time_safe_stop = p.time
                    print('safe stop ' + str(p.time) + " [s]")

                    pos, vel, acc = polynomialRef(p.comPoseW_des, p.x0,
                                                  p.comTwistW_des, np.zeros(6),
                                                  p.comAccW_des, np.zeros(6),
                                                  (1 / 4) * (1 / test['freq']))
                    state += 1

            elif state == 3:
                if p.time - time_safe_stop < (1/4)*(1/test['freq']):
                    if test['profile']:
                        pr.enable()
                    time = p.time - time_safe_stop
                    p.comPoseW_des, p.comTwistW_des, p.comAccW_des = p.Hframe2World(pos(time), vel(time), acc(time))

                    p.Wcom2Joints_des()
                    
                    p.WBC(p.comPoseW_des,
                          p.comTwistW_des,
                          p.comAccW_des,
                          comControlled=True,
                          type=test['typeWBC'])
                    if test['profile']:
                        pr.disable()

                else:
                    state += 1

                    p.pid.setPDjoints(p.kp_j, p.kd_j, p.ki_j)
                    p.q_des = p.q.copy()
                    p.qd_des[:] = 0.
                    print('restore PDs ' + str(p.time) +" [s]")

            else:
                p.gravityCompensation()

            # send desired command to the ros controller
            p.send_command()



    except (ros.ROSInterruptException, ros.service.ServiceException):

        ros.signal_shutdown("killed")
        p.deregister_node()
    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

        if test['profile']:
            ps = pstats.Stats(pr)
            ps.dump_stats(os.environ['LOCOSIM_DIR'] + '/landing_controller/go1tests/stats_wbc.cprof')
            # then go in go1test with the terminal and run
            # pyprof2calltree -k -i stats_wbc.cprof

        if conf.plotting:
            plotFrame('position', time_log=p.time_log, des_Pose_log=p.comPoseW_des_log, Pose_log=p.comPoseW_log,
                      title='CoM', frame='W', sharex=True, sharey=False, start=0,end=-1)
            plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.comTwistW_des_log, Twist_log=p.comTwistW_log,
                      title='CoM', frame='W', sharex=True, sharey=False, start=0, end=-1)
            plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_des_log, Pose_log=p.basePoseW_log,
                      title='base', frame='W', sharex=True, sharey=False, start=0, end=-1)
            plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_des_log, Twist_log=p.baseTwistW_log,
                      title='base', frame='W', sharex=True, sharey=False, start=0, end=-1)

            plotContacts('position', time_log=p.time_log, des_LinPose_log=p.B_contacts_des_log, LinPose_log=p.B_contacts_log,
                         contact_states=p.contact_state_log, frame='B', sharex=True, sharey=False, start=0, end=-1)

            plotContacts('position', time_log=p.time_log, des_LinPose_log=p.W_contacts_des_log,
                         LinPose_log=p.W_contacts_log,
                         contact_states=p.contact_state_log, frame='W', sharex=True, sharey=False, start=0, end=-1)

            plotContacts('velocity', time_log=p.time_log,  des_LinTwist_log=p.B_vel_contacts_des_log,
                          frame='B', sharex=True, sharey=False, start=0, end=-1)

            plotContacts('GRFs', time_log=p.time_log, des_Forces_log=p.grForcesW_des_log,
                         Forces_log=p.grForcesW_log, contact_states=p.contact_state_log, frame='W',
                         sharex=True, sharey=False, start=0, end=-1)

            plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log,sharex=True, sharey=False,
                      start=0, end=-1)
            plotJoint('velocity', time_log=p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log, sharex=True,
                      sharey=False, start=0, end=-1)
            plotJoint('torque', time_log=p.time_log, tau_log=p.tau_log, tau_ffwd_log = p.tau_ffwd_log,
                      tau_des_log=p.tau_fb_log, sharex=True, sharey=False, start=0, end=-1)

            plotWrenches('fb', 8, p.time_log, des_Wrench_fb_log=p.wrench_fbW_log)
            plotWrenches('ffwd', 9, p.time_log, des_Wrench_ffwd_log=p.wrench_ffW_log)
            plotWrenches('g', 10, p.time_log, des_Wrench_g_log=p.wrench_gW_log)

            plotFrameLinear('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_legOdom_log,
                            Twist_log=p.baseLinTwistImuW_log, title='Base velocity estimate', frame='W', sharex=True,
                            sharey=False, start=0,end=-1)

