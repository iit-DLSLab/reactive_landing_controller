#!/usr/bin/env python3

import numpy as np
import time
import timeit
import os

import pygo1_hal as pal
from pygo1_hal import LowLevelInterface
from pygo1_hal import LowState
from pygo1_hal import LowCmd

#  np.set_printoptions(suppress=True, precision=3, linewidth=120)
np.set_printoptions(suppress=True, precision=3, linewidth=os.get_terminal_size(0)[0])

robot_interface = LowLevelInterface()

state = LowState()
lowcmd = LowCmd()
vectcmd = LowCmd()
vectcmd = np.zeros(60, dtype=np.float32)

zero_command = np.zeros(60, dtype=np.float32)

# TODO: Do we want to send this before the user press enter?
robot_interface.send_command(zero_command)  # Send zero command to receive data

motor_idxs = [
    pal.FL_0, pal.FL_1, pal.FL_2,
    pal.FR_0, pal.FR_1, pal.FR_2,
    pal.RL_0, pal.RL_1, pal.RL_2,
    pal.RR_0, pal.RR_1, pal.RR_2,
]

sin_mid_q = np.array([
    0.0, 1.2, -2.0,
    0.0, 1.2, -2.0,
    0.0, 1.2, -2.0,
    0.0, 1.2, -2.0,
])

ff_torques = [
    +0.65, 0., 0.,
    -0.65, 0., 0.,
    +0.65, 0., 0.,
    -0.65, 0., 0.,
]

Kp = [
   5., 5., 5.,
   5., 5., 5.,
   5., 5., 5.,
   5., 5., 5.,
]

Kd = [
   1., 1., 1.,
   1., 1., 1.,
   1., 1., 1.,
   1., 1., 1.,
]

motion_time = 0
rate_count = 0
sin_count = 0

qinit = np.zeros(12, dtype=np.float32)
qdes = np.zeros(12, dtype=np.float32)


def joint_linear_interpolation(initpos, targetpos, rate):
    rate = np.clip(rate, 0., 1.)
    p = initpos*(1-rate) + targetpos*rate
    return p


def leg_sin_delta(c):
    sin_joint0 = 0.
    sin_joint1 = 0.6 * np.sin(3*np.pi*c/1000.0)
    #  sin_joint1 = 0.
    sin_joint2 = -0.6 * np.sin(1.8*np.pi*c/1000.0)
    #  sin_joint2 = 0

    return np.array([sin_joint0, sin_joint1, sin_joint2])


print("Testing")
input("Press Enter to continue...")


for i in range(25000):
    try:
        starttime = timeit.default_timer()

        # TODO: Print takes a lot of time!
        #  print("\33[h\33[2j")
        os.system('clear')
        print(f"counter: {i}")
        print("*---------------------------------------------*", flush=False)
        print("imu state:")
        print(f"rpy : {state.imu.rpy[0]:+.3f}, {state.imu.rpy[1]:+.3f}, {state.imu.rpy[2]:+.3f}", flush=False)
        print(f"quaternion : {state.imu.quaternion[0]:+.3f}, {state.imu.quaternion[1]:+.3f}, {state.imu.quaternion[2]:+.3f}, {state.imu.quaternion[3]:+.3f}", flush=False)
        print(f"gyroscope : {state.imu.gyroscope[0]:+.3f}, {state.imu.gyroscope[1]:+.3f}, {state.imu.gyroscope[2]:+.3f}", flush=False)
        print(f"accelerometer : {state.imu.accelerometer[0]:+.3f}, {state.imu.accelerometer[1]:+.3f}, {state.imu.accelerometer[2]:+.3f}", flush=False)
        print("*---------------------------------------------*", flush=False)
        print("motorState state:")
        print(f"lf : {state.motorState[3].q:+.3f}, {state.motorState[4].q:+.3f}, {state.motorState[5].q:+.3f} | {state.motorState[3].dq:+.3f}, {state.motorState[4].dq:+.3f}, {state.motorState[5].dq:+.3f}", flush=False)
        print(f"rf : {state.motorState[0].q:+.3f}, {state.motorState[1].q:+.3f}, {state.motorState[2].q:+.3f} | {state.motorState[0].dq:+.3f}, {state.motorState[1].dq:+.3f}, {state.motorState[2].dq:+.3f}", flush=False)
        print(f"lh : {state.motorState[9].q:+.3f}, {state.motorState[10].q:+.3f}, {state.motorState[11].q:+.3f} | {state.motorState[9].dq:+.3f}, {state.motorState[10].dq:+.3f}, {state.motorState[11].dq:+.3f}", flush=False)
        print(f"rh : {state.motorState[6].q:+.3f}, {state.motorState[7].q:+.3f}, {state.motorState[8].q:+.3f} | {state.motorState[6].dq:+.3f}, {state.motorState[7].dq:+.3f}, {state.motorState[8].dq:+.3f}", flush=False)
        print("")

        state = robot_interface.receive_observation()

        motion_time += 1

        #  lowcmd.motorCmd[pal.FR_0].tau = -0.65
        #  lowcmd.motorCmd[pal.FL_0].tau = +0.65
        #  lowcmd.motorCmd[pal.RR_0].tau = -0.65
        #  lowcmd.motorCmd[pal.RL_0].tau = +0.65

        # First get record initial position
        if 0 <= motion_time < 10:
            qinit[:] = [state.motorState[idx].q for idx in motor_idxs]

        # Second, move to the origin point of a sine movement
        if 10 <= motion_time < 400:
            rate_count += 1
            rate = rate_count/200.0

            for ii in range(len(qdes)):
                qdes[ii] = joint_linear_interpolation(qinit[ii], sin_mid_q[ii], rate)

        # Last, do sine wave
        if motion_time >= 400:
            sin_count += 1

            #  for ii in [2]:
            for ii in range(4):
                qdes[ii*3:(ii*3)+3] = sin_mid_q[ii*3:(ii*3)+3] + leg_sin_delta(sin_count)

        # TODO: This operation is very slow!
        #  for ii, idx in enumerate(motor_idxs):
        #      lowcmd.motorCmd[idx].q = qdes[ii]
        #      lowcmd.motorCmd[idx].dq = 0.
        #      lowcmd.motorCmd[idx].Kp = Kp[ii]
        #      lowcmd.motorCmd[idx].Kd = Kd[ii]
        #      lowcmd.motorCmd[idx].tau = ff_torques[ii]
        #  robot_interface.send_low_command(lowcmd)
        #  print(qdes[5])

        for ii, idx in enumerate(motor_idxs):
            vectcmd[idx*5] = qdes[ii]  # q
            vectcmd[idx*5+1] = 0.  # dq
            vectcmd[idx*5+2] = Kp[ii]  # Kp
            vectcmd[idx*5+3] = Kd[ii]  # Kd
            vectcmd[idx*5+4] = ff_torques[ii]  # Kd
            #  print(idx*5, ii)
        robot_interface.send_command(vectcmd)

        elapsed_time = timeit.default_timer() - starttime
        time.sleep(max(1/1000. - elapsed_time, 0))


    except KeyboardInterrupt:
        break

print("Elapsed time:", elapsed_time)
