from base_controllers.quadruped_controller import Controller
import rospy as ros
from base_controllers.utils.common_functions import *
import base_controllers.params as conf

import os

import cProfile, pstats, io
from pstats import SortKey



ROBOT_NAME = 'go1'
profiler_filename = os.environ['LOCOSIM_DIR'] + '/laning_controller/tests/stats_basic.cprof'


if __name__ == '__main__':
    p = Controller(ROBOT_NAME)
    
    pr = cProfile.Profile()

    p.startController(world_name='fast.world', additional_args=['go0_conf:=standDown'])
    p.startupProcedure()

    try:
        while not ros.is_shutdown():
            pr.enable()
            p.updateKinematics()
            pr.disable()
            p.send_command(p.q_des, np.zeros(p.robot.na), p.self_weightCompensation())
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        
        
    ps = pstats.Stats(pr)
    ps.dump_stats(profiler_filename)
    # then go in go1test with the terminal and run
    # pyprof2calltree -k -i stats_new.cprof

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
