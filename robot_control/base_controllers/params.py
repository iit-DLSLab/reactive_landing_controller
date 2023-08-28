# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

robot_params = {}
robot_params['hyq'] = {'dt': 0.004,
                        'kp': np.array([400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400]),
                        'kd': np.array([6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6]),
                        'q_0':  np.array([-0.2, 0.7, -1.4,  -0.2, -0.7, 1.4, -0.2, 0.7, -1.4, -0.2, -0.7, 1.4]),
                        'kp_wbc': np.array([15., 15., 15.]*4),#np.array([10., 10., 10.]*4),
                        'kd_wbc': np.array([1., 1., 1.]*4),#np.array([1., 1., 1.]*4),
                        'ki_wbc': np.array([0., 0., 0.]*4),#np.array([0.3, 0.3, 0.3]*4),
                        # virtual impedance wrench control
                        'kp_lin': np.array([2000., 2000., 2000.]),  # x y z
                        'kd_lin': np.array([200., 200., 200.]),
                        'kp_ang': np.array([1000., 1000., 1000.]),  # R P Y
                        'kd_ang': np.array([100., 100., 100.]),
                        'joint_names': ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',
                                        'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',
                                        'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',
                                        'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'],
                        'ee_frames': ['lf_foot', 'lh_foot', 'rf_foot','rh_foot'],
                        'force_th': 50,
                        'spawn_x': 0.0,
                        'spawn_y': 0.0,
                        'spawn_z': 0.8,
                        'spawn_R': 0.0,
                        'spawn_P': 0.0,
                        'spawn_Y': np.pi/2,
                       'buffer_size': 30001}
robot_params['solo'] ={'dt': 0.002,
                       'kp': [5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5.],
                       'kd': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                       'q_0':  np.array([0,  np.pi/4, -np.pi/2,    0, -np.pi/4,  np.pi/2, -0,  np.pi/4, -np.pi/2, -0, -np.pi/4,  np.pi/2]),
                       'q_fold': np.array([0,  1.57, -3.13, 0,    1.57, -3.13, 0, -1.57, 3.13, 0, -1.57, 3.13]),
                       'joint_names': ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',
                                       'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',
                                       'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',
                                       'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'],
                       'ee_frames': ['lf_foot', 'lh_foot', 'rf_foot','rh_foot'],
                       'real_robot': False,
                       'force_th': 2,
                       'spawn_x': 0.0,
                       'spawn_y': 0.0,
                       'spawn_z': 0.3,
                        'buffer_size': 1501} # note the frames are all aligned with base for joints = 0
robot_params['solo_fw'] ={'dt': 0.002,
                        'kp': [5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 0, 0],
                        'kd': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0, 0],
                        'joint_names':  ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',
                                         'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',
                                         'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',
                                         'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint',
                                         'z_left_wheel_joint', 'z_right_wheel_joint'],
                        'q_0':  np.array([0.,  np.pi/4, -np.pi/2,  0., -np.pi/4,  np.pi/2, -0.,  np.pi/4, -np.pi/2, 0., -np.pi/4,  np.pi/2, 0., 0.]),
                        'ee_frames': ['lf_foot', 'lh_foot', 'rf_foot','rh_foot'],
                        'real_robot': False,
                        'force_th': 4,
                        'spawn_x': 0.0,
                        'spawn_y': 0.0,
                        'spawn_z': 0.3,
                        'buffer_size': 1501} # note the frames are all aligned with base for joints = 0

robot_params['aliengo'] ={'dt': 0.002,
                        'kp': 60.*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]),
                        'kd': 10.*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]),
                        'kp_real': 15.*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]),
                        'kd_real': 0.5*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]),
                        #'q_0':  np.array([0.2, 0.7, -1.4, -0.2, 0.7, -1.4, 0.2, 0.7, -1.4, -0.2, 0.7, -1.4]),
                        'q_0':  np.array([0.2, 0.78, -1.7,   -0.20, 0.78, -1.7, 0.20, 0.78, -1.7, -0.20, 0.78, -1.7]),
                        #'q_fold': np.array([0.6, 1.7, -2.77, 0.6, 1.7, -2.77, -0.6, 1.52, -2.77, -0.6, 1.52, -2.77]),
                        'q_fold': np.array([0.2, 1.7, -2.7,  -0.2, 1.7, -2.7, 0.2, 1.7, -2.7, -0.2, 1.7, -2.7]),
                        'joint_names': ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',
                                       'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',
                                       'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',
                                       'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'],
                        'ee_frames': ['lf_foot', 'lh_foot', 'rf_foot','rh_foot'],
                        'real_robot': False,
                        'force_th': 10,
                          'spawn_x': 0.0,
                          'spawn_y': 0.0,
                          'spawn_z': 0.5,
                          'buffer_size': 30001} # note the frames are all aligned with base for joints = 0
robot_params['go1'] ={'dt': 0.002,
                      'buffer_size': 60001, # 120 seconds
                      # simulation gains
                      # stand alone joint pid
                      'kp': np.array([15., 15., 15.]*4),
                      'kd': np.array([1., 1., 1.]*4),
                      'ki': np.array([0., 0., 0.]*4),
                      # joint pid + wbc (optional)
                      'kp_wbc': np.array([15., 15., 15.]*4),#np.array([10., 10., 10.]*4),
                      'kd_wbc': np.array([1., 1., 1.]*4),#np.array([1., 1., 1.]*4),
                      'ki_wbc': np.array([0., 0., 0.]*4),#np.array([0.3, 0.3, 0.3]*4),
                      # virtual impedance wrench control
                      'kp_lin': np.array([800, 500., 900.]),  # x y z
                      'kd_lin': np.array([100, 100., 100.]),
                      'kp_ang': np.array([100, 100., 100.]),  # R P Y
                      'kd_ang': np.array([10., 10., 10.]),
                      # real robot gains
                      # stand alone joint pid
                      'kp_real': np.array([30., 30.,30.]*4),
                      'kd_real': np.array([.3, .3, .3]*4),
                      'ki_real': np.array([1.5, 1.5, 1.5]*4),
                      # joint pid + wbc (optional)
                      'kp_wbc_real': np.array([20., 30., 40.]*4),
                      'kd_wbc_real': np.array([.3, .3, .3]*4),
                      'ki_wbc_real': np.array([1.5, 1.5, 1.5]*4),
                      # virtual impedance wrench control
                      # 'kp_lin_real': np.array([300, 300., 300.]), # x y z
                      # 'kd_lin_real': np.array([30., 30., 30.]),
                      # 'kp_ang_real': np.array([50, 50., 50.]), # R P Y
                      # 'kd_ang_real': np.array([10., 10., 10.]),
                      'kp_lin_real': np.array([300., 200., 350.]), #np.array([300., 300., 400.]), # x y z
                      'kd_lin_real': np.array([40., 40., 80.]), #np.array([30., 20., 60.]),
                      'kp_ang_real': np.array([40., 80., 40.]), # #np.array([30., 50., 30.]), # R P Y
                      'kd_ang_real': np.array([3., 5., 3.]), #np.array([2., 4., 2.]),
                      # joint configuration
                      'q_0':  np.array([0.2, 0.78, -1.7,  0.2, 0.78, -1.7, -0.2, 0.78, -1.7, -0.2, 0.78, -1.7]),
                      'q_fold': np.array([0.2, 1.7, -2.7, 0.2,  1.7, -2.7, -0.2, 1.7,  -2.7, -0.2, 1.7, -2.7]),
                      'joint_names': ['lf_haa_joint',  'lf_hfe_joint', 'lf_kfe_joint',
                                      'lh_haa_joint',  'lh_hfe_joint', 'lh_kfe_joint',
                                      'rf_haa_joint',  'rf_hfe_joint', 'rf_kfe_joint',
                                      'rh_haa_joint',  'rh_hfe_joint', 'rh_kfe_joint'],
                      # ee params
                      'ee_frames': ['lf_foot', 'lh_foot', 'rf_foot','rh_foot'],
                      'force_th': 18.,
                      'contact_th': 0.01,
                      # simulation spawn [m] and [rad]
                      'spawn_x': 0.0,
                      'spawn_y': 0.0,
                      'spawn_z': .27,
                      'spawn_R': 0.0,
                      'spawn_P': 0.0,
                      'spawn_Y': 0.0,
                      # use real robot or simulation
                      'real_robot': False} # note the frames are all aligned with base for joints = 0

robot_params['ur5'] ={'dt': 0.001, 
                       'kp': np.array([300, 300, 300,30,30,1]),
                       'kd':  np.array([20,20,20,5, 5,0.5]),
                       #'q_0':  np.array([ 0.3, -1.3, 1.0, -0.7, 0.7, 0.5]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                       'q_0':  np.array([ -0.3223527113543909,-0.7805794638446351, -2.48,-1.6347843609251917, -1.5715253988849085, -1.0017417112933558]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                       'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
                       'ee_frame': 'tool0',
                       'control_mode': 'point', # 'trajectory','point'
                       'real_robot': False,
                       'control_type': 'position', # 'position', 'torque'
                       'gripper_sim': True,
                       'spawn_x' : 0.5,
                       'spawn_y' : 0.35,
                       'spawn_z' : 1.75,
                       'buffer_size': 50000} # note the frames are all aligned with base for joints = 0

robot_params['jumpleg'] ={'dt': 0.001,
                       'kp': np.array([100, 100, 100, 10, 10, 10 ]),
                       'kd':  np.array([10,10,10, 0.2,0.2,0.2]),
                       'q_0':  np.array([ 0.0, 0.0, 0.25, -0.24010055475883635,0.7092776153747403,-1.4185292429491714]),
                       'joint_names': ['base_x_joint', 'base_y_joint', 'base_z_joint', 'lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint'],
                       'ee_frame': 'lf_foot',
                       'spawn_x' : 0.0,
                       'spawn_y' : 0.0,
                       'spawn_z' : 0.0,
                       'buffer_size': 1000} # note the frames are all aligned with base for joints = 0

robot_params['climbingrobot'] ={'dt': 0.001,
                       'kp': np.array([0 ,    0,    400,  100,    50,   50,    50, 30, 30]),
                       'kd':  np.array([0,    0,    10,   10,     10,   10,     4,   4, 4  ]),

                       'q_0':  np.array([ 0, 0, 8.0, 0,0,0, -1.57, 0.0, 0.0 ]),
                       'joint_names': ['mountain_wire_pitch', 'mountain_wire_roll',  'wire_base_prismatic',
                                       'wire_base_pitch', 'wire_base_roll','wire_base_yaw',
                                       'hip_pitch', 'hip_roll', 'knee'],
                       'ee_frame': 'foot',
                       'spawn_x' : 0.0,
                       'spawn_y' : 0.0,
                       'spawn_z' : 20.0, 
                       'buffer_size': 10000} # note the frames are all aligned with base for joints = 0

robot_params['climbingrobot_slider'] ={'dt': 0.001,
                       'kp': np.array([400, 0 ,   0,    400,  100,    50,   50,    50, 30, 30]),
                       'kd':  np.array([10, 0,    0,    10,   10,     10,   10,     4,   4, 4 ]),

                       'q_0':  np.array([ 0.0, 0, 0, 8.0, 0,0,0, -1.57, 0.0, 0.0 ]),
                       'joint_names': ['slider', 'mountain_wire_pitch', 'mountain_wire_roll',  'wire_base_prismatic',
                                       'wire_base_pitch', 'wire_base_roll','wire_base_yaw',
                                       'hip_pitch', 'hip_roll', 'knee'],
                       'ee_frame': 'foot',
                       'spawn_x' : 0.0,
                       'spawn_y' : 0.0,
                       'spawn_z' : 20.0,
                       'buffer_size': 10000} # note the frames are all aligned with base for joints = 0

robot_params['climbingrobot2'] ={'dt': 0.001,
                       'kp': np.array([0 ,    0,    400,  40,    40,   40,
                                       0 ,    0,    400,  40,    40,   40,
                                       50, 30, 30]),
                       'kd':  np.array([20,    20,    100,   20,     20,   20,
                                        20,    20,    100,   20,     20,   20,
                                        10,   10, 10  ]),
                       # this corresposnds to p = [0.03, 2.5, -6] from matlab WF  which is located in anchor_pos1 and correspods to
                       'q_0':  np.array([ 0.0,    1.17  ,  4.0000   ,      0. ,   -1.17 ,        0.,
                                          0.0,  -1.17,    4.0000  ,       0. ,  1.17,           0.,
                                          -1.57, 0.0, 0.0 ]),
                       # 'q_0': np.array([0.0, 0, 4, 0, 0.0, 0,
                       #                    0.0, 0, 4, 0, 0, 0,
                       #                    -1.57, 0.0, 0.0]),
                       'Ko': np.array([10, 10, 10]),
                       'Do': np.array([0.01, 0.01, 0.01]),
                       'joint_names': ['mountain_wire_pitch_r', 'mountain_wire_roll_r',  'wire_base_prismatic_r',
                                       'wire_base_pitch_r', 'wire_base_roll_r','wire_base_yaw_r',
                                        'mountain_wire_pitch_l', 'mountain_wire_roll_l',  'wire_base_prismatic_l',
                                       'wire_base_pitch_l', 'wire_base_roll_l','wire_base_yaw_l',
                                       'hip_pitch', 'hip_roll', 'knee'],
                       'ee_frame': 'foot',
                       'spawn_x' : 0.3,
                       'spawn_y' : 0.0,
                       'spawn_z' : 20.0,
                       'spawn_2x': 0.3,
                       'spawn_2y': 5.0,
                       'spawn_2z': 20.0,
                       'buffer_size': 10000} # note the frames are all aligned with base for joints = 0

robot_params['climbingrobot2landing'] ={'dt': 0.001,
                       'kp': np.array([0 ,    0,    400,  40,    40,   40,
                                       0 ,    0,    400,  40,    40,   40,
                                       50, 30, 30,
                                       30, 30., 30,30.
                                       ]),
                       'kd':  np.array([20,    20,    100,   20,     20,   20,
                                        20,    20,    100,   20,     20,   20,
                                        10,   10, 10,
                                        10, 3, 10 ,3
                                        ]),
                       # this corresposnds to p = [0.03, 2.5, -6] from matlab WF  which is located in anchor_pos1
                       'q_0':  np.array([ 0.0,    1.17  ,  4.0000   ,      0. ,   -1.17 ,        0.,
                                          0.0,  -1.17,    4.0000  ,       0. ,  1.17,           0.,
                                          -1.57, 0.0, 0.0,
                                          0., -0, 0.,0.
                                          ]),
                       'Ko': np.array([100, 100, 100]),
                       'Do': np.array([10, 10, 10]),
                       'joint_names': ['mountain_wire_pitch_r', 'mountain_wire_roll_r',  'wire_base_prismatic_r',
                                       'wire_base_pitch_r', 'wire_base_roll_r','wire_base_yaw_r',
                                        'mountain_wire_pitch_l', 'mountain_wire_roll_l',  'wire_base_prismatic_l',
                                       'wire_base_pitch_l', 'wire_base_roll_l','wire_base_yaw_l',
                                       'hip_pitch', 'hip_roll', 'knee',
                                       'hip_yaw_landing_l', 'wheel_joint_l', 'hip_yaw_landing_r', 'wheel_joint_r'],
                       'ee_frame': 'foot',
                       'spawn_x' : 0.2,
                       'spawn_y' : 0.0,
                       'spawn_z' : 20.0,
                       'spawn_2x': 0.2,
                       'spawn_2y': 5.0,
                       'spawn_2z': 20.0,
                       'buffer_size': 10000} # note the frames are all aligned with base for joints = 0

#
robot_params['starbot'] ={'dt': 0.001,
                       'kp': 50.*np.array([50. ,   50.,    50.,  50.0 ,
                       			10. ,   10.,    10.,  10. ,
                                100. ,   100.,    100.,  100. ,
                                10. ,   10.,    10.,  10. ,
                       			0.05 ,   0.05,    0.05,  0.05]),
                       'kd':   1.0*np.array([2.,    2.,    2.,   2. ,
                       			1.,    1.,    1.,   1. ,
                                5.,    5.,   5.,   5. ,
                                1.,    1.,    1.,   1. ,
                       			1,   1,    1,  1 ]),

                       'q_0':  np.array([ 0, 0, 0, 0,
                       			0., 0., 0., 0.0,
                                0., 0., 0., 0.,
                                0., 0., 0., 0.0,
                                0., 0., 0., 0.0 ]),
                       'joint_names': ['lf_bs_joint',       'lh_bs_joint',  'rf_bs_joint', 'rh_bs_joint',
                       			      'lf_upper_leg_joint', 'lh_upper_leg_joint',  'rf_upper_leg_joint', 'rh_upper_leg_joint',
                       			      'lf_lower_leg_joint', 'lh_lower_leg_joint',  'rf_lower_leg_joint', 'rh_lower_leg_joint',
                                      'lf_pre_wheel_joint', 'lh_pre_wheel_joint',  'rf_pre_wheel_joint', 'rh_pre_wheel_joint',
                                      'lf_wheel_joint', 'lh_wheel_joint',  'rf_wheel_joint', 'rh_wheel_joint'],
                       'ee_frames': ['lf_wheel','lh_wheel','rf_wheel','rh_wheel'],
                       'force_th': 20.,
                       'spawn_x' : 0.0,
                       'spawn_y' : 0.0,
                       'spawn_z' : 1.0,
                       'buffer_size': 10000} # note the frames are all aligned with base for joints = 0

verbose = False
plotting = False


