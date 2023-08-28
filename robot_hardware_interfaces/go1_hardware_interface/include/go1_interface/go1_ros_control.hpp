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
#ifndef GO1_ROS_CONTROL_H
#define GO1_ROS_CONTROL_H

#include <memory>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "go1_interface/go1_robot_hw.hpp"

namespace go12ros {

class Go1ROSControl {
public:

    std::string CLASS_NAME = "Go1ROSControl";

	Go1ROSControl();
	~Go1ROSControl();

  /** @brief init */
	void init();

  /** @brief update */
	void update(const ros::Time& time, const ros::Duration& period);

private:

  /** @brief ROS node handle */
	std::shared_ptr<ros::NodeHandle> node_handle_;
	
  /** @brief Go1 Hardware interface */
	std::shared_ptr<Go1RobotHw> robot_hw_;

  /** @brief controller_manager provides the infrastructure to load, unload, start and stop controllers */
	std::shared_ptr<controller_manager::ControllerManager> controller_manager_;




};

} // namespace


#endif
