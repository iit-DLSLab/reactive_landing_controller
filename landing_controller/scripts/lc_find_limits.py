from base_controllers.quadruped_controller import Controller
from landing_controller.controller.landingManager import LandingManager
from landing_controller.settings import SETTINGS # simumation details are in SETTINGS['SIMS']
from landing_controller.controller.utility import *


ROBOT_NAME = 'go1'  # go1, solo, (aliengo)
world_name = 'slow.world'
use_gui = False

phase_deg_list = np.arange(-180, 180, 30)
#magnitude_init_list = np.ones_like(phase_deg_list)

if __name__ == '__main__':
    setSavePath(SETTINGS, 'simulations')

    if SETTINGS['save_log']:
        stdout = sys.stdout
        logfile = open(SETTINGS['save_path'] + "/log.txt", "w")

    try:
        p = Controller(ROBOT_NAME)
        if SETTINGS['VIDEO']['save']:
            world_name = 'camera_'+world_name

        p.startController(world_name=world_name,
                          use_ground_truth_pose=True,
                          use_ground_truth_contacts=False,
                          additional_args=['gui:=true',
                                           'go0_conf:=standDown',
                                           'pid_discrete_implementation:=true'])

        p.startupProcedure()


        lm = LandingManager(p, SETTINGS)
        lm.p.setWBCConstraints()
        sim_counter = -1
        if SETTINGS['save_log']:
            sys.stdout = logfile


        for simulation in SETTINGS['SIMS']:
            if 'find_limits' not in simulation['name']:
                sys.stdout = stdout
                assert simulation['name'] != 'find_limits', "You should use lc_simulation.py. Exit..."
            elif simulation['name'] == 'find_limits_linear':
                sim_counter += 1
                sim_str = findLimitsInitCond2str(simulation)
                print(sim_str)
                for naive in [True, False]:
                    if naive:
                        ctrl = 'naive approach'
                        ctrl_str= 'na'
                    else:
                        ctrl = 'LC'
                        ctrl_str = 'lc'
                    for ii, phase_deg in enumerate(phase_deg_list):
                        print( "Searching maximum magnitude for base height "+ str(simulation['pose'][2])+ " m for phase "
                               + str(phase_deg) + " deg with " + ctrl, flush=True)
                        phase_rad = phase_deg * np.pi / 180
                        unit_baseTwist = np.array([np.cos(phase_rad), np.sin(phase_rad), 0., 0., 0., 0.])
                        n_test = 31
                        magnitude_try = np.linspace(0, 3., n_test)
                        magnitude_succeed = []
                        fail_couter = 0
                        for test_id in range(n_test):

                            simulation_try = {'name': simulation['name'],
                                              'pose': simulation['pose'],
                                              'twist':  magnitude_try[test_id] * unit_baseTwist,
                                              'useWBC': simulation['useWBC'],
                                              'useIK': simulation['useIK'],
                                              'typeWBC': simulation['typeWBC'],  # or 'qp' (used only if useWBC is True)
                                              'id': '',
                                              't_video': 0.0,
                                              'directory': ''}
                            setId(simulation_try, test_id)
                            simulation_try['id'] = str(sim_counter) + ctrl_str + str(phase_deg) + str(test_id)
                            SETTINGS['SIMS'][sim_counter] = simulation_try

                            print("--> (sim" +simulation_try['id']+") Testing magnitude: " + str(np.around(magnitude_try[test_id] , 1)) + " [m/s] with " + ctrl, flush=True)

                            ret = lm.run(sim_counter,
                                         simulation_try['pose'],
                                         simulation_try['twist'],
                                         simulation_try['useIK'],
                                         simulation_try['useWBC'],
                                         simulation_try['typeWBC'],
                                         naive=naive)

                            if ret:
                                print('    Succeed')
                                magnitude_succeed.append(np.around(magnitude_try[test_id] , 1))
                                fail_couter = 0
                            else:
                                print('    Failed')
                                fail_couter += 1 # number of subsequent fails

                            if fail_couter > 3:
                                break

                        print('#' * 60)
                        print("Succeeding magnitudes for " + str(phase_deg) + " deg with " + ctrl + ":\n" + str(magnitude_succeed) + " [m/s]", flush=True)
                        print('#' * 60)
            elif simulation['name'] == 'find_limits_angular':
                sim_counter += 1
                simtry_counter = 0
                sim_str = findLimitsInitCond2str_angularTest(simulation)
                print(sim_str)

                while True:
                    # all in rad or in rad/s
                    simtry_counter += 1
                    simulation['pose'][3:6] = 0
                    simulation['twist'][3:6] = 0
                    if simulation['test_limit'] == '+roll':
                        simulation['pose'][3] = simtry_counter * 5 * DEG2RAD
                    elif simulation['test_limit'] == '-roll':
                        simulation['pose'][3] = -simtry_counter * 5 * DEG2RAD
                    elif simulation['test_limit'] == '+pitch':
                        simulation['pose'][4] = simtry_counter * 5 * DEG2RAD
                    elif simulation['test_limit'] == '-pitch':
                        simulation['pose'][4] = -simtry_counter * 5 * DEG2RAD
                    elif simulation['test_limit'] == '+omegax':
                        simulation['twist'][3] = simtry_counter * 5 * DEG2RAD
                    elif simulation['test_limit'] == '-omegax':
                        simulation['twist'][3] = -simtry_counter * 5 * DEG2RAD
                    elif simulation['test_limit'] == '+omegay':
                        simulation['twist'][4] = simtry_counter * 5 * DEG2RAD
                    elif simulation['test_limit'] == '-omegay':
                        simulation['twist'][4] = -simtry_counter * 5 * DEG2RAD
                    elif simulation['test_limit'] == '+omegaz':
                        simulation['twist'][5] = simtry_counter * 5 * DEG2RAD
                    elif simulation['test_limit'] == '-omegaz':
                        simulation['twist'][5] = -simtry_counter * 5 * DEG2RAD

                    simulation_try = {'name': simulation['name'],
                                      'pose': simulation['pose'],
                                      'twist': simulation['twist'],
                                      'useWBC': simulation['useWBC'],
                                      'useIK': simulation['useIK'],
                                      'typeWBC': simulation['typeWBC'],  # or 'qp' (used only if useWBC is True)
                                      'test_limit': simulation['test_limit'],
                                      'id': '',
                                      't_video': 0.0,
                                      'directory': ''}
                    setId(simulation_try, simtry_counter)
                    simulation_try['id'] = str(sim_counter) + simulation_try['id']
                    SETTINGS['SIMS'][sim_counter] = simulation_try

                    print("--> (sim " + simulation_try['id']+") Testing orientation: " + str(simulation_try['pose'][3:]) + " [rad] and angular velocity "
                              + str(simulation_try['twist'][3:])+ " [rad/s]", flush=True)

                    ret = lm.run(sim_counter,
                                 simulation_try['pose'],
                                 simulation_try['twist'],
                                 simulation_try['useIK'],
                                 simulation_try['useWBC'],
                                 simulation_try['typeWBC'],
                                 naive=False)

                    if ret:
                        print('    Succeed')


                    else:
                        print('    Failed')
                        break

                if 'roll' in simulation_try['test_limit']:
                    roll_rad = simulation_try['pose'][3] - np.sign(simulation_try['pose'][3]) * 5 *DEG2RAD
                    roll_deg = np.round(roll_rad/DEG2RAD, 4)
                    print('Limit ' + simulation_try['test_limit'] +' '+ str( np.round(roll_rad, 4) )+ " [rad] = " +str(roll_deg) + " [deg]")

                elif 'pitch' in simulation_try['test_limit']:
                    pitch_rad =  simulation_try['pose'][4] - np.sign(simulation_try['pose'][4]) * 5 *DEG2RAD
                    pitch_deg = np.round(pitch_rad/DEG2RAD, 4)
                    print('Limit ' + simulation_try['test_limit'] +' '+ str( np.round(pitch_rad, 4) )+ " [rad] = " +str(pitch_deg) + " [deg]")

                elif 'omegax' in simulation_try['test_limit']:
                    omegax_rad = simulation_try['twist'][3] - np.sign(simulation_try['twist'][3]) * 5 * DEG2RAD
                    omegax_deg = np.round(omegax_rad/ DEG2RAD, 4)
                    print('Limit ' + simulation_try['test_limit'] +' '+ str( np.round(omegax_rad, 4) ) + " [rad/s] = " + str(omegax_deg) + " [deg/s]")

                elif 'omegay' in simulation_try['test_limit']:
                    omegay_rad = simulation_try['twist'][4] - np.sign(simulation_try['twist'][4]) * 5 * DEG2RAD
                    omegay_deg = np.round(omegay_rad / DEG2RAD, 4)
                    print('Limit ' + simulation_try['test_limit'] +' '+ str( np.round(omegay_rad, 4) ) + " [rad/s] = " + str(omegay_deg) + " [deg/s]")

                elif 'omegaz' in simulation_try['test_limit']:
                    omegaz_rad = simulation_try['twist'][5] - np.sign(simulation_try['twist'][5]) * 5 * DEG2RAD
                    omegaz_deg = np.round(omegaz_rad / DEG2RAD, 4)
                    print('Limit ' + simulation_try['test_limit'] +' '+ str( np.round(omegaz_rad, 4) ) + " [rad/s] = " + str(omegaz_deg) + " [deg/s]")



    except (ros.ROSInterruptException, ros.service.ServiceException) as e:
        if SETTINGS['save_log']:
            logfile.close()
            sys.stdout = stdout
        print(e)
    finally:
        if SETTINGS['save_log']:
            logfile.close()
            sys.stdout = stdout
        ros.signal_shutdown("killed")
        p.deregister_node()
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")



        # store all the init conds
        if SETTINGS['INIT_CONDS']['save_all']:
            saveAllInitConds(SETTINGS['save_path'], SETTINGS['SIMS'], SETTINGS['VIDEO']['speedUpDown'], SETTINGS['verbose'])
