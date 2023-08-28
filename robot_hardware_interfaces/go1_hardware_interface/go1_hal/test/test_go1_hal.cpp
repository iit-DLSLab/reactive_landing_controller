// This file sends sinusoidal (joint position) signals to the motors. File based on unitree's position example.

#include "go1_hal/go1_hal.h"
// #include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <iomanip>

namespace unitree = UNITREE_LEGGED_SDK;


double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
		double p;
		rate = std::min(std::max(rate, 0.0), 1.0);
		p = initPos*(1-rate) + targetPos*rate;
		return p;
}

std::array<double, 3> legSinDelta(double counter)
{
	std::array<double, 3> sin_joints;

	sin_joints[0] = 0.0;
	// sin_joints[1] = 0.0;
	sin_joints[1] = 0.6 * sin(3*M_PI*counter/1000.0);
	// sin_joints[2] = 0.0;
	sin_joints[2] = -0.6 * sin(1.8*M_PI*counter/1000.0);

	return sin_joints;
}

void print_state(go1hal::LowState state)
{
		std::cout << "\33[H\33[2J" <<'\n'; //clear screen
		std::cout << std::fixed << std::setprecision(4);
		//std::cout << "Counter: " << ii << '\n';
		
		std::cout << "*---------------------------------------------*" << '\n';
		std::cout << "IMU state:" << '\n';
		std::cout << "RPY : " << state.imu.rpy[0] << ", " << state.imu.rpy[1] << ", " << state.imu.rpy[2] << "." << '\n';
		std::cout << "Quaternion : " << state.imu.quaternion[0] << ", " << state.imu.quaternion[1] << ", " << state.imu.quaternion[2] << ", " << state.imu.quaternion[3] << "." << '\n';
		std::cout << "Gyroscope : " << state.imu.gyroscope[0] << ", " << state.imu.gyroscope[1] << ", " << state.imu.gyroscope[2] << "." << '\n';
		std::cout << "Acceleration : " << state.imu.accelerometer[0] << ", " << state.imu.accelerometer[1] << ", " << state.imu.accelerometer[2] << "." << '\n';
		
		std::cout << "*---------------------------------------------*" << '\n';
		std::cout << "Joint  state:" << '\n';
		std::cout << "LF : " << state.motorState[3].q << ", " << state.motorState[4].q	<< ", " << state.motorState[5].q << '\n';
		std::cout << "RF : " << state.motorState[0].q << ", " << state.motorState[1].q	<< ", " << state.motorState[2].q  << '\n';
		std::cout << "LH : " << state.motorState[9].q << ", " << state.motorState[10].q  << ", " << state.motorState[11].q << '\n';
		std::cout << "RH : " << state.motorState[6].q << ", " << state.motorState[7].q	<< ", " << state.motorState[8].q << '\n';
		

		std::cout << "*---------------------------------------------*" << '\n';
		std::cout << "Joint  velocity:" << '\n';
		std::cout << "LF : " << state.motorState[3].dq << ", " << state.motorState[4].dq  << ", " << state.motorState[5].dq << '\n';
		std::cout << "RF : " << state.motorState[0].dq << ", " << state.motorState[1].dq  << ", " << state.motorState[2].dq << '\n';
		std::cout << "LH : " << state.motorState[9].dq << ", " << state.motorState[10].dq	<< ", " << state.motorState[11].dq << '\n';
		std::cout << "RH : " << state.motorState[6].dq << ", " << state.motorState[7].dq  << ", " << state.motorState[8].dq << '\n';

		std::cout << "*---------------------------------------------*" << '\n';
		std::cout << "Joint  torque:" << '\n';
		std::cout << "LF : " << state.motorState[3].tauEst << ", " << state.motorState[4].tauEst  << ", " << state.motorState[5].tauEst << '\n';
		std::cout << "RF : " << state.motorState[0].tauEst << ", " << state.motorState[1].tauEst  << ", " << state.motorState[2].tauEst << '\n';
		std::cout << "LH : " << state.motorState[9].tauEst << ", " << state.motorState[10].tauEst	<< ", " << state.motorState[11].tauEst << '\n';
		std::cout << "RH : " << state.motorState[6].tauEst << ", " << state.motorState[7].tauEst  << ", " << state.motorState[8].tauEst << '\n';
		
		
		std::cout << std::endl;

}
	


int main()
{
	std::cout << "Testing Go1-hal. Check the robot is suspended in the air!" << std::endl
						<< "Press Enter to continue..." << std::endl;
	std::cin.ignore();

	go1hal::LowLevelInterface robot_interface;
	// std::shared_ptr<go1hal::LowLevelInterface> robot_interface;
	// robot_interface = std::make_shared<go1hal::LowLevelInterface>(go1hal::LowLevelInterface());
	// unitree::InitEnvironment(); // memory lock -> Requires sudo

	go1hal::LowState state = {0};
	go1hal::LowCmd lowcmd = {0};
	std::array<float, 60> zero_command = {0};
 

 	usleep(300000);
	// Send a 'dummy' command
	robot_interface.SendCommand(zero_command);
	
	float qInit[12] = {0};
	float qDes[12] = {0};

	int motorIdxs[12] = {
		go1hal::FL_0, go1hal::FL_1, go1hal::FL_2,
		go1hal::FR_0, go1hal::FR_1, go1hal::FR_2,
		go1hal::RL_0, go1hal::RL_1, go1hal::RL_2,
		go1hal::RR_0, go1hal::RR_1, go1hal::RR_2,
	};
		
	float sin_mid_q[12] = {
		0.0, 1.2, -2.0,
		0.0, 1.2, -2.0,
		0.0, 1.2, -2.0,
		0.0, 1.2, -2.0,
	};

	float ff_torques[12] = {
		+0.65, 0.0, 0.0,
		-0.65, 0.0, 0.0,
		+0.65, 0.0, 0.0,
		-0.65, 0.0, 0.0,
	};

	float Kp[12] = {0};

	float Kd[12] = {0};

	int motiontime = 0; // Counter for total motion time
	int rate_count = 0; // Counter for moving to mid of the
	int sin_count = 0;
	int loop_ms = 1; // Milliseconds

	int total_steps = 10000;

	for (int tt = 0; tt <= total_steps; ++tt)
	{
		state = robot_interface.ReceiveObservation();

		motiontime++;

		// Sleep for some time
		std::this_thread::sleep_for(std::chrono::milliseconds(loop_ms));
		
		// First, get record initial position
		if (motiontime >= 0 && motiontime < 1000)
		{
			for (int jj = 0; jj < 12; ++jj)
			{
				qInit[jj] = state.motorState[motorIdxs[jj]].q;
			}
		}

		// NOTE: We set stiffness and damping now because it's not a good idea to set them if we didn't define qDes before.
		if (motiontime >= 1000)
		{
			// std::cout << std::endl;
			std::fill_n(Kp, 12, 5);
			std::fill_n(Kd, 12, 1);
		}

		// Second, move to the origin point of a sine movement with Kp Kd
		// if (motiontime >= 500 && motiontime < 1500){
		if (motiontime >= 1000 && motiontime < 4000)
		{
			rate_count++;
			double rate = rate_count/3000.0;	// needs count to 200

			// std::cout << rate << "(" << rate_count << ")" <<std::endl;

			for (int jj = 0; jj < 12; ++jj)
			{
				qDes[jj] = jointLinearInterpolation(qInit[jj], sin_mid_q[jj], rate);
				// std::cout << " % " << qInit[jj];
			}
		}

		// Last, do sine wave
		if( motiontime >= 4000){
			sin_count++;

			for (int ll = 0; ll < 4; ++ll)
			{
				std::array<double, 3> sin_delta = legSinDelta(sin_count);
				for (int ii = 0; ii < 3; ++ii)
				{
					qDes[3*ll+ii] = sin_mid_q[3*ll+ii] + sin_delta[ii];
				}
			}
		}

		// Set desired position
		for (int ii = 0; ii < 12; ++ii)
		{
			lowcmd.motorCmd[motorIdxs[ii]].mode = 0x0A;  // motor switch to servo (PMSM) mode
			lowcmd.motorCmd[motorIdxs[ii]].q = qDes[ii];
			lowcmd.motorCmd[motorIdxs[ii]].dq = 0.0;
			lowcmd.motorCmd[motorIdxs[ii]].Kp = Kp[ii];
			lowcmd.motorCmd[motorIdxs[ii]].Kd = Kd[ii];
			lowcmd.motorCmd[motorIdxs[ii]].tau = ff_torques[ii];
		}
		
		// std::cout << tt << " | " << lowcmd.motorCmd[motorIdxs[1]].q << " vs " << state.motorState[motorIdxs[1]].q << "(" << qInit[1] << ")" << std::endl;
		// std::cout << tt << " / " << total_steps << std::endl;


		lowcmd.head[0] = 0xFE;
		lowcmd.head[1] = 0xEF;
		lowcmd.levelFlag = unitree::LOWLEVEL;
		robot_interface.SendLowCmd(lowcmd);
		if (tt%100 == 0)
		{
			std::cout << "Counter: " << tt << " / " << total_steps << std::endl;
			print_state(state);
		}
	}

	return 0;


}

