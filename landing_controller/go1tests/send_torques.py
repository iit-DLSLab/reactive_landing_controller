# This script sends only  constant ffwd torques to the robot and waits for Ctrl+C to close.
# For using real robot, you have to modify the conf options.

from base_controllers.quadruped_controller import Controller
import rospy as ros
import time

from base_controllers.utils.pidManager import PidManager

ROBOT_NAME = 'go1'                         # go1, solo, (aliengo)


if __name__ == '__main__':
    p = Controller(ROBOT_NAME)
    p.startController(use_ground_truth_pose=False, use_ground_truth_contacts=False)
    p.pid = PidManager(p.joint_names)
    p.pid.setPDjoints(np.zeros(p.robot.na),
                      np.zeros(p.robot.na),
                      np.zeros(p.robot.na))
    q_des = p.q_des.copy()
    qd_des = np.zeros(12)
    tau_ffwd = np.array([0.35,0,0, 0,0,0, 0,0,0, 0,0,0])

    try:
        while p.imu_utils.counter < p.imu_utils.timeout:
            p.updateKinematics()
            p.imu_utils.IMU_bias_estimation(p.b_R_w, p.B_imu_lin_acc)
            p.tau_ffwd[:] = 0.
            p.send_command(q_des, qd_des, p.tau_ffwd)
        print('Imu bias estimation completed')
        while not ros.is_shutdown():
            p.updateKinematics()
            p.imu_utils.compute_lin_vel(p.W_base_lin_acc, p.loop_time)

            p.send_command(q_des, qd_des, tau_ffwd)

    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()


