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

#ifndef GO1_ROBOT_HW_H
#define GO1_ROBOT_HW_H

#include <base_hardware_interface/base_robot_hw.h>
#include <go1_hal/go1_hal.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

typedef Eigen::Matrix<double, 6,1> butterFilterParams;  


namespace go12ros
{

class Go1RobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface
{
public:
  Go1RobotHw();
  virtual ~Go1RobotHw();

  void init();
  void read();
  void write();

private:

  /** @brief Map Go1 internal joint indices to WoLF joints order */
  std::array<unsigned int, 12> go1_motor_idxs_
          {{
          go1hal::FL_0, go1hal::FL_1, go1hal::FL_2, // LF
          go1hal::RL_0, go1hal::RL_1, go1hal::RL_2, // LH
          go1hal::FR_0, go1hal::FR_1, go1hal::FR_2, // RF
          go1hal::RR_0, go1hal::RR_1, go1hal::RR_2, // RH
          }};

  /** @brief Go1-HAL */
  go1hal::LowLevelInterface go1_interface_;
  go1hal::LowState go1_state_ = {0};
  go1hal::LowCmd go1_lowcmd_ = {0};

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();


  /** @brief IMU realtime publisher */
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> imu_acc_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> imu_euler_pub_;
  std::vector<butterFilterParams> velocityFilterBuffer;
  void filt(const double raw, butterFilterParams & filt);

  std::vector<double> imu_euler_raw_;
  std::vector<double> imu_orientation_raw_;
  std::vector<double> remove_euler_;
  std::vector<double> remove_quaternion_;
  bool is_remove_yaw_set_ = false;

};

}

#endif
