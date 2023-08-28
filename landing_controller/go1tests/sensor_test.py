# This script reads from the robot sensors and waits for Ctrl+C to close.
# For using real robot, you have to modify the conf options.

from base_controllers.quadruped_controller import Controller
import rospy as ros
import time
from base_controllers.utils.common_functions import *

from base_controllers.utils.pidManager import PidManager
import base_controllers.params as conf

ROBOT_NAME = 'go1'                         # go1, solo, (aliengo)


if __name__ == '__main__':
    p = Controller(ROBOT_NAME)
    p.startController(use_ground_truth_pose=True,
                      use_ground_truth_contacts=False,
                      additional_args=['gui:=false',
                                       'go0_conf:=standDown',
                                       'pid_discrete_implementation:=true'])
    p.pid = PidManager(p.joint_names)
    p.pid.setPDjoints(np.zeros(p.robot.na),
                      np.zeros(p.robot.na),
                      np.zeros(p.robot.na))
    q_des = p.q_des.copy()
    qd_des = np.zeros(12)
    tau_ffwd = np.zeros(12)

    try:
        while p.imu_utils.counter < p.imu_utils.timeout:
            p.updateKinematics()
            p.imu_utils.IMU_bias_estimation(p.b_R_w, p.baseLinAccB)
            p.tau_ffwd[:] = 0.
            p.send_command(q_des, qd_des, tau_ffwd)
        print('Imu bias estimation compleated')
        while not ros.is_shutdown():
            p.updateKinematics()

            p.send_command(q_des, qd_des, tau_ffwd)

    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()



    except (ros.ROSInterruptException, ros.service.ServiceException):

        ros.signal_shutdown("killed")
        p.deregister_node()

    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

        if conf.plotting:
            plotFrame('position', time_log=p.time_log, des_Pose_log=p.comPoseW_des_log, Pose_log=p.comPoseW_log,
                      title='CoM', frame='W', sharex=True, sharey=False, start=0, end=-1)
            plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.comTwistW_des_log, Twist_log=p.comTwistW_log,
                      title='CoM', frame='W', sharex=True, sharey=False, start=0, end=-1)
            plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_des_log, Pose_log=p.basePoseW_log,
                      title='base', frame='W', sharex=True, sharey=False, start=0, end=-1)
            plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_des_log, Twist_log=p.baseTwistW_log,
                      title='base', frame='W', sharex=True, sharey=False, start=0, end=-1)

            plotContacts('position', time_log=p.time_log, des_LinPose_log=p.B_contacts_des_log,
                         LinPose_log=p.B_contacts_log,
                         contact_states=p.contact_state_log, frame='B', sharex=True, sharey=False, start=0, end=-1)

            plotContacts('position', time_log=p.time_log, des_LinPose_log=p.W_contacts_des_log,
                         LinPose_log=p.W_contacts_log,
                         contact_states=p.contact_state_log, frame='W', sharex=True, sharey=False, start=0, end=-1)

            plotContacts('velocity', time_log=p.time_log, des_LinTwist_log=p.B_vel_contacts_des_log,
                         frame='B', sharex=True, sharey=False, start=0, end=-1)

            plotContacts('GRFs', time_log=p.time_log, des_Forces_log=p.grForcesW_des_log,
                         Forces_log=p.grForcesW_log, contact_states=p.contact_state_log, frame='W',
                         sharex=True, sharey=False, start=0, end=-1)

            plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, sharex=True, sharey=False,
                      start=0, end=-1)
            plotJoint('velocity', time_log=p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log, sharex=True,
                      sharey=False, start=0, end=-1)
            plotJoint('torque', time_log=p.time_log, tau_log=p.tau_log, tau_ffwd_log=p.tau_ffwd_log,
                      tau_des_log=p.tau_fb_log, sharex=True, sharey=False, start=0, end=-1)

            plotWrenches('fb', 8, p.time_log, des_Wrench_fb_log=p.wrench_fbW_log)
            plotWrenches('ffwd', 9, p.time_log, des_Wrench_ffwd_log=p.wrench_ffW_log)
            plotWrenches('g', 10, p.time_log, des_Wrench_g_log=p.wrench_gW_log)

            plotFrameLinear('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_legOdom_log,
                            Twist_log=p.baseLinTwistImuW_log, title='Base velocity estimate', frame='W', sharex=True,
                            sharey=False, start=0, end=-1)

            plotFrameLinear('velocity', time_log=p.time_log,
                            Twist_log=p.baseLinAccB_log, title='accelerometer', frame='B', sharex=True,
                            sharey=False, start=0, end=-1)
