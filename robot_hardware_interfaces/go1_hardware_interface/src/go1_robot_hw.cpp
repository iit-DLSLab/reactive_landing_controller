/*
 * Copyright (C) 2022 Francesco Roscia
 * Author: Francesco Roscia
 * email:  francesco.roscia@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include "go1_interface/go1_robot_hw.hpp"

namespace go12ros
{

using namespace hardware_interface;

int64_t utime_now() {

    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    if (timeofday.tv_sec < 0 || timeofday.tv_sec > UINT_MAX)
        throw std::runtime_error("Timeofday is out of dual signed 32-bit range");
    uint32_t sec	= timeofday.tv_sec;
    uint32_t nsec = timeofday.tv_usec * 1000;

    return (int64_t) (((uint64_t)sec)*1000000 + ((uint64_t)nsec) / 1000);
}

Go1RobotHw::Go1RobotHw()
{

    robot_name_ = "go1";
}

Go1RobotHw::~Go1RobotHw()
{

}

void Go1RobotHw::init()
{


    // Hardware interfaces: Joints
    auto joint_names = loadJointNamesFromSRDF();
    if(joint_names.size()>0)
    {
      WolfRobotHwInterface::initializeJointsInterface(joint_names);
      registerInterface(&joint_state_interface_);
      registerInterface(&joint_effort_interface_);
      velocityFilterBuffer.resize(joint_names.size());
      for (unsigned int i = 0; i < joint_names.size(); i++)
    	{
	       //init velocity filters
	       velocityFilterBuffer[i][0] = 0.0;
	       velocityFilterBuffer[i][1] = 0.0;
	       velocityFilterBuffer[i][2] = 0.0;
	       velocityFilterBuffer[i][3] = 0.0;
	       velocityFilterBuffer[i][4] = 0.0;
	       velocityFilterBuffer[i][5] = 0.0;
	    }

    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register joint interface.");
      return;
    }

    // Hardware interfaces: IMU
    auto imu_name = loadImuLinkNameFromSRDF();
    if(!imu_name.empty())
    {
      WolfRobotHwInterface::initializeImuInterface(imu_name);
      registerInterface(&imu_sensor_interface_);
      imu_euler_raw_.resize(3);
      imu_euler_raw_[0] = 0.0;
      imu_euler_raw_[1] = 0.0;
      imu_euler_raw_[2] = 0.0;

      imu_orientation_raw_.resize(4);
      imu_orientation_raw_[0] = 1.0;
      imu_orientation_raw_[1] = 0.0;
      imu_orientation_raw_[2] = 0.0;
      imu_orientation_raw_[3] = 0.0;

      remove_euler_.resize(3);
      remove_euler_[0] = 0.0;
      remove_euler_[1] = 0.0;
      remove_euler_[2] = 0.0;
      remove_quaternion_.resize(4);
      remove_quaternion_[0] = 1.0;
      remove_quaternion_[1] = 0.0;
      remove_quaternion_[2] = 0.0;
      remove_quaternion_[3] = 0.0;
    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register imu interface.");
      return;
    }

    go1_interface_.InitCmdData(go1_lowcmd_);
    startup_routine();

    ros::NodeHandle root_nh;
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh,	"/go1/ground_truth", 1));
    imu_acc_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(root_nh,	"/go1/trunk_imu", 1));
    imu_euler_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(root_nh,	"/go1/euler_imu", 1));

}


void Go1RobotHw::filt(const double raw, butterFilterParams & buffer)
{
//notch filter cut-off 50 Hz; BW 20 Hz.
//double a[3] = {1.0000000e+00,  -1.7633 ,  0.8541};
//double b[3] = {0.9270 ,  -1.7633 ,  0.9270};
//notch filter cut-off 110 Hz; BW 220 Hz.
//double a[3] = {1.0000000e+00,  -0.8433 ,  0.0945};
//double b[3] = {0.5473 ,  -0.8433 ,  0.5473};
//2nd order butterworth cut 150 for fs = 1000 Hz
//double a[3] = {1.0000000e+00,  -0.7478 ,  0.2722};
//double b[3] = {0.1311 ,  0.2622 ,  0.1311};
//2nd order butterworth cut 150 for fs = 2000 Hz
//double a[3] = {1.0000 ,  -1.349 ,  0.5140};
//double b[3] = {0.0413 ,  0.0825 ,  0.0413};
//2nd order butterworth cut 150 for fs = 800 Hz
//double a[3] = {1.0000 ,  -0.4629 ,  0.2097};
//double b[3] = {0.1867 ,   0.3734 ,  0.1867};
//2nd order butterworth cut 250 for fs = 1000 Hz
//    double a[3] = { 1.0000, 0.0000, 0.1716 };
//    double b[3] = { 0.2929, 0.5858, 0.2929 };
//2nd order butterworth cut 40
//double a[3] = {1.0000000e+00,  -3.6952738e-01 ,  1.9581571e-01};
//double b[3] = {2.0657208e-01 ,  4.1314417e-01 ,  2.0657208e-01};
//2nd order butterworth 30
double a[3] = {1.0000000e+00,  -7.4778918e-01,   2.7221494e-01};
double b[3] = {1.3110644e-01 ,  2.6221288e-01,   1.3110644e-01};
//2nd order butterworth 15
//double a[3] = {1.0000000e+00 , -1.3489677e+00 ,  5.1398189e-01 };
//double b[3] = {4.1253537e-02 ,  8.2507074e-02,   4.1253537e-02};
//2nd order butterworth 8
//double a[3] = {1.0000000e+00 , -1.6474600e+00 ,  7.0089678e-01 };
//double b[3] = {1.3359200e-02 ,  2.6718400e-02 ,  1.3359200e-02  };
//first 3 elements are y0 y1 y2 second 3 x0 x1 x2
    int input = 3;
    buffer[input + 0] = raw;
    buffer[0] = -a[1] * buffer[1] - a[2] * buffer[2] + b[0] * buffer[input + 0]
            + b[1] * buffer[input + 1] + b[2] * buffer[input + 2];

    buffer[input + 2] = buffer[input + 1];
    buffer[input + 1] = buffer[input + 0];
    buffer[input] = buffer[2];
    buffer[2] = buffer[1];
    buffer[1] = buffer[0];
}

void Go1RobotHw::read()
{
    // Get robot data
    go1_state_ = go1_interface_.ReceiveObservation();

    // ------
    // Joints
    // ------
    for (unsigned int jj = 0; jj < n_dof_; ++jj)
    {
        joint_position_[jj] = static_cast<double>(go1_state_.motorState[go1_motor_idxs_[jj]].q)     ;

	//filt(static_cast<double>(go1_state_.motorState[go1_motor_idxs_[jj]].dq) , velocityFilterBuffer[jj]);
        joint_velocity_[jj] = static_cast<double>(go1_state_.motorState[go1_motor_idxs_[jj]].dq) ;//velocityFilterBuffer[jj][0];

        joint_effort_[jj]   = static_cast<double>(go1_state_.motorState[go1_motor_idxs_[jj]].tauEst);
    }

    // ---
    // IMU
    // ---
  
  if (not is_remove_yaw_set_)
    {
      // These lines remove init yaw of the robot
      remove_euler_[2] = -static_cast<double>(go1_state_.imu.rpy[2]);
      remove_quaternion_[0] = cos(remove_euler_[2]/2); // w
      // remove_quaternion[1] = 0.                     // x
      // remove_quaternion[2] = 0.                     // y
      remove_quaternion_[3] = sin(remove_euler_[2]/2); // z
      is_remove_yaw_set_ = true;
    }
    imu_orientation_raw_[0] = static_cast<double>(go1_state_.imu.quaternion[0]);  // w
    imu_orientation_raw_[1] = static_cast<double>(go1_state_.imu.quaternion[1]);  // x
    imu_orientation_raw_[2] = static_cast<double>(go1_state_.imu.quaternion[2]);  // y
    imu_orientation_raw_[3] = static_cast<double>(go1_state_.imu.quaternion[3]);  // z

    imu_orientation_[0] = remove_quaternion_[0] * imu_orientation_raw_[0] - remove_quaternion_[3] * imu_orientation_raw_[3];
    imu_orientation_[1] = remove_quaternion_[0] * imu_orientation_raw_[1] - remove_quaternion_[3] * imu_orientation_raw_[2];
    imu_orientation_[2] = remove_quaternion_[0] * imu_orientation_raw_[2] + remove_quaternion_[3] * imu_orientation_raw_[1];
    imu_orientation_[3] = remove_quaternion_[0] * imu_orientation_raw_[3] + remove_quaternion_[3] * imu_orientation_raw_[0];


    imu_euler_raw_[0] = static_cast<double>(go1_state_.imu.rpy[0]);  // R
    imu_euler_raw_[1] = static_cast<double>(go1_state_.imu.rpy[1]);  // P
    imu_euler_raw_[2] = static_cast<double>(go1_state_.imu.rpy[2]);  // Y
    imu_euler_[0] = imu_euler_raw_[0] + remove_euler_[0];
    imu_euler_[1] = imu_euler_raw_[1] + remove_euler_[1];
    imu_euler_[2] = imu_euler_raw_[2] + remove_euler_[2];

    imu_ang_vel_[0] = static_cast<double>(go1_state_.imu.gyroscope[0]);
    imu_ang_vel_[1] = static_cast<double>(go1_state_.imu.gyroscope[1]);
    imu_ang_vel_[2] = static_cast<double>(go1_state_.imu.gyroscope[2]);

    imu_lin_acc_[0] = static_cast<double>(go1_state_.imu.accelerometer[0]);
    imu_lin_acc_[1] = static_cast<double>(go1_state_.imu.accelerometer[1]);
    imu_lin_acc_[2] = static_cast<double>(go1_state_.imu.accelerometer[2]);


    // Publish the IMU data NOTE: missing covariances
    if(odom_pub_.get() && odom_pub_->trylock())
    {
      odom_pub_->msg_.pose.pose.orientation.w         = imu_orientation_[0];
      odom_pub_->msg_.pose.pose.orientation.x         = imu_orientation_[1];
      odom_pub_->msg_.pose.pose.orientation.y         = imu_orientation_[2];
      odom_pub_->msg_.pose.pose.orientation.z         = imu_orientation_[3];
      odom_pub_->msg_.twist.twist.angular.x    = imu_ang_vel_[0];
      odom_pub_->msg_.twist.twist.angular.y    = imu_ang_vel_[1];
      odom_pub_->msg_.twist.twist.angular.z    = imu_ang_vel_[2];

      odom_pub_->msg_.header.stamp = ros::Time::now();
      odom_pub_->unlockAndPublish();
    }


    if(imu_acc_pub_.get() && imu_acc_pub_->trylock())
    {
      imu_acc_pub_->msg_.x = imu_lin_acc_[0];
      imu_acc_pub_->msg_.y = imu_lin_acc_[1];
      imu_acc_pub_->msg_.z = imu_lin_acc_[2];
      
      imu_acc_pub_->unlockAndPublish();
    }

    if(imu_euler_pub_.get() && imu_euler_pub_->trylock())
    {
      imu_euler_pub_->msg_.x = imu_euler_[0];
      imu_euler_pub_->msg_.y = imu_euler_[1];
      imu_euler_pub_->msg_.z = imu_euler_[2];
      
      imu_euler_pub_->unlockAndPublish();
    }
}

void Go1RobotHw::write()
{

    for (unsigned int jj = 0; jj < n_dof_; ++jj)
    {
      go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].mode = 0x0A;  // motor switch to servo (PMSM) mode
      go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].tau = static_cast<float>(joint_effort_command_[jj]  );
      //these to be sure to have pure torque control mode
      go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].q = unitree::PosStopF; 
      go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].Kp = 0;
      go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].dq = unitree::VelStopF; 
      go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].Kd = 0;
    }
    go1_lowcmd_.head[0] = 0xFE;
		go1_lowcmd_.head[1] = 0xEF;
		go1_lowcmd_.levelFlag = unitree::LOWLEVEL;

    go1_interface_.SendLowCmd(go1_lowcmd_);
}

void Go1RobotHw::send_zero_command()
{
    std::array<float, 60> zero_command = {0};
    // go1_interface_->SendCommand(zero_command);
    //IMPORTANT! this ensures all the Kp Kd gains are set to zero
    go1_interface_.SendCommand(zero_command);
}

void Go1RobotHw::startup_routine()
{
    send_zero_command();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

} // namespace
