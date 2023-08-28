import numpy as np
#######################
# Simulation Settings #
#######################
# SIMS_SETTINGS is a list of dictionary containing the following fields:
# 'name' (str):             name of the jump, used for generate folders containing plots
# 'pose' (np.ndarray(6,)):  initial pose of the robot base
# 'twist'(np.ndarray(6,)):  initial twist of the robot base
# 'useWBC' (bool):          use wbc in landing phase (at least one between useWBC and useIK must be true)
# 'useIK'(bool):            use ik in landing phase
# 'typeWBC' (str):          'projection' or 'qp' (used only if useWBC is True)
# 'id' (str):               numeric identifier of the simulation, will be filled at run-time
# 't_video' (float):        start time in the video, will be filled at run-time
# directory (str):          directory for storing plots, will be filled at run-time

DEG2RAD = np.pi / 180
SIMS_SETTINGS = []
SIMS_SETTINGS.append({'name': 'A_high_jump',
                      'pose': np.array([0., 0., .6, 0., 0., 0]),
                      'twist': np.array([1., 0.0, 0., 0., 0., 0.]),
                      'useWBC': True,
                      'useIK': False,
                      'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
                      'id': '',
                      'directory': ''})
# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., .6, 0., 0., 0]),
#                       'twist': np.array([1.0, 0.0, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., .6, 0., 0., 0.]),
#                       'twist': np.array([0., 1.0, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., .6, 0., 0., 0.]),
#                       'twist': np.array([0., -1.0, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., 1., 0., 0, 0.]),
#                       'twist': np.array([-1., 0., 0., 0., 0.5, 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'A_high_jump',
#                       'pose': np.array([0., 0., 0.85, 0., 0., 0.]),
#                       'twist': np.array([0.5, 0., 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'B_lateral_mix',
#                       'pose': np.array([0., 0., 0.6, 0., 0., 0.]),
#                       'twist': np.array([0.3, 0.5, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': True,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'B_lateral_mix',
#                       'pose': np.array([0., 0., 0.85, 0., 0., 0.]),
#                       'twist': np.array([0.3, 0.6, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'C_pitch',
#                       'pose': np.array([0., 0., 0.6, 0., -20. * DEG2RAD, 0.]),
#                       'twist': np.array([1.0, 0., 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'C_pitch',
#                       'pose': np.array([0., 0., 0.6, 0., +20. * DEG2RAD, 0.]),
#                       'twist': np.array([1.0, 0., 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
#
# SIMS_SETTINGS.append({'name': 'D_pitch_mix',
#                       'pose': np.array([0., 0., 0.6, 0., -10. * DEG2RAD, 0.]),
#                       'twist': np.array([0.5, 0.5, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'D_pitch_mix',
#                       'pose': np.array([0., 0., 0.6, 0., +10. * DEG2RAD, 0.]),
#                       'twist': np.array([0.5, 0.5, 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'E_omega',
#                       'pose': np.array([0., 0., .6, 0., 0., 0.]),
#                       'twist': np.array([1.0, 0., 0., 0., 1., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'E_omega',
#                       'pose': np.array([0., 0., .6, 0., 0., 0.]),
#                       'twist': np.array([1.0, 0., 0., 0., -1., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
# SIMS_SETTINGS.append({'name': 'F_roll',
#                       'pose': np.array([0., 0., 0.6, 0., -10. * DEG2RAD, 0.]),
#                       'twist': np.array([1.0, 0., 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'F_roll',
#                       'pose': np.array([0., 0., 0.6, 0., +10. * DEG2RAD, 0.]),
#                       'twist': np.array([1.0, 0., 0., 0., 0., 0.]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})

# SIMS_SETTINGS.append({'name': 'find_limits_angular',
#                       'pose': np.array([0., 0., 0.6, 5*DEG2RAD, 0., 0.]),
#                       'twist': np.array([1, 0., 0., 0., 0., 0.]),
#                       'test_limit': '+roll',
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_angular',
#                       'pose': np.array([0., 0., 0.6, -5*DEG2RAD, 0., 0.]),
#                       'twist': np.array([1, 0., 0., 0., 0., 0.]),
#                       'test_limit': '-roll',
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_angular',
#                       'pose': np.array([0., 0., 0.6, 0., 5*DEG2RAD, 0.]),
#                       'twist': np.array([1, 0., 0., 0., 0., 0.]),
#                       'test_limit': '+pitch',
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_angular',
#                       'pose': np.array([0., 0., 0.6, 0., -5*DEG2RAD, 0.]),
#                       'twist': np.array([1, 0., 0., 0., 0., 0.]),
#                       'test_limit': '-pitch',
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_angular',
#                       'pose': np.array([0., 0., 0.6, 0., 0., 0.]),
#                       'twist': np.array([1, 0., 0., 5*DEG2RAD, 0., 0.]),
#                       'test_limit': '+omegax',
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_angular',
#                       'pose': np.array([0., 0., 0.6, 0., 0., 0.]),
#                       'twist': np.array([1, 0., 0., -5*DEG2RAD, 0., 0.]),
#                       'test_limit': '-omegax',
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_angular',
#                       'pose': np.array([0., 0., 0.6, 0., 0., 0.]),
#                       'twist': np.array([1, 0., 0., 0., 5*DEG2RAD, 0.]),
#                       'test_limit': '+omegay',
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_angular',
#                       'pose': np.array([0., 0., 0.6, 0., 0., 0.]),
#                       'twist': np.array([1, 0., 0., 0., -5*DEG2RAD, 0.]),
#                       'test_limit': '-omegay',
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_angular',
#                       'pose': np.array([0., 0., 0.6, 0., 0., 0.]),
#                       'twist': np.array([1, 0., 0., 0., 0., 5*DEG2RAD]),
#                       'test_limit': '+omegaz',
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_angular',
#                       'pose': np.array([0., 0., 0.6, 0., 0., 0.]),
#                       'twist': np.array([1, 0., 0., 0., 0,-5*DEG2RAD]),
#                       'test_limit': '-omegaz',
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'projection',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_linear',
#                       'pose': np.array([0., 0., 0.35, 0., 0., 0.]),
#                       'twist': np.array([0, 0., 0., 0., 0., 0.]),
#                       'magnitude_init_list':  0.5*np.ones(12), #np.array([1.5, 1.4, 1.7, 2.2, 1.9, 2.2, 2.8, 2.2, 1.9, 2.1, 1.7, 1.4]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'qp',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
# SIMS_SETTINGS.append({'name': 'find_limits_linear',
#                       'pose': np.array([0., 0., 0.5, 0., 0., 0.]),
#                       'twist': np.array([0, 0., 0., 0., 0., 0.]),
#                       'magnitude_init_list':  0.5*np.ones(12), #np.array([1.5, 1.4, 1.7, 2.2, 1.9, 2.2, 2.8, 2.2, 1.9, 2.1, 1.7, 1.4]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'qp',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})
#
#
#
#
# SIMS_SETTINGS.append({'name': 'find_limits_linear',
#                       'pose': np.array([0., 0., 0.8, 0., 0., 0.]),
#                       'twist': np.array([0, 0., 0., 0., 0., 0.]),
#                       'magnitude_init_list':  0.1*np.ones(12),#np.array([3.0, 3.3, 2.7, 2.3, 2.1, 2.1, 2.2, 2.1, 2.0, 2.3, 2.7, 3.3]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'qp',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                      'directory': ''})
#
#
#
# SIMS_SETTINGS.append({'name': 'find_limits_linear',
#                       'pose': np.array([0., 0., 1., 0., 0., 0.]),
#                       'twist': np.array([0, 0., 0., 0., 0., 0.]),
#                       'magnitude_init_list':  0.5*np.ones(12),#np.array([2.6, 2.2, 1.6, 1.3, 1.7, 1.3, 2.2, 1.1, 1.7, 1.4, 1.6, 2.1]),
#                       'useWBC': True,
#                       'useIK': False,
#                       'typeWBC': 'qp',  # or 'qp' (used only if useWBC is True)
#                       'id': '',
#                       'directory': ''})