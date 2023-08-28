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

        #noise = None
        noise = {'horz_vel_init':Normal(0, 0.4, 2),
                 'qd':Normal(0, 0.1, 12),
                 'tau':Normal(0, 0.3, 12)}

        lm = LandingManager(p, SETTINGS, noise)
        sim_counter = -1
        if SETTINGS['save_log']:
            sys.stdout = logfile

        print("Noises description")
        for k in noise.keys():
            print(k + f": Normal(mean={noise[k].mean}, std={noise[k].std}, size={noise[k].size})")

        for simulation in SETTINGS['SIMS']:
            if 'find_limits' not in simulation['name']:
                sys.stdout = stdout
                assert simulation['name'] != 'find_limits', "You should use lc_simulation.py. Exit..."
            elif simulation['name'] == 'find_limits_linear':
                sim_counter += 1

                sim_str = findLimitsInitCond2str(simulation)
                print(sim_str)

                n_test = 21
                n_phases = len(phase_deg_list)
                nums_success = np.zeros([n_phases, n_test])

                for ii, phase_deg in enumerate(phase_deg_list):
                    print( "Computing success rate with base height "+ str(simulation['pose'][2])+ " m", flush=True)
                    phase_rad = phase_deg * np.pi / 180
                    unit_baseTwist = np.array([np.cos(phase_rad), np.sin(phase_rad), 0., 0., 0., 0.])

                    magnitude_try = np.linspace(0, 4., n_test)
                    for test_id in range(n_test):
                        if phase_deg != -180 and magnitude_try[test_id] == 0:
                            nums_success[ii, test_id] = nums_success[0, 0]
                            continue

                        simulation_try = {'name': simulation['name'],
                                          'pose': simulation['pose'],
                                          'twist':  magnitude_try[test_id] * unit_baseTwist,
                                          'useWBC': simulation['useWBC'],
                                          'useIK': simulation['useIK'],
                                          'typeWBC': simulation['typeWBC'],  # or 'qp' (used only if useWBC is True)
                                          'id': '',
                                          't_video': 0.0,
                                          'directory': ''}
                        print("-->  Testing magnitude: " + str(np.around(magnitude_try[test_id], 1)) + " [m/s] and phase "
                           + str(phase_deg) + " deg" , flush=True)

                        for jj in range(10):
                            setId(simulation_try, test_id)
                            simulation_try['id'] +=  str(phase_deg) + str(jj)
                            SETTINGS['SIMS'][sim_counter] = simulation_try
                            print(f'    Run #{jj}')

                            ret = lm.run(0,
                                         simulation_try['pose'],
                                         simulation_try['twist'],
                                         simulation_try['useIK'],
                                         simulation_try['useWBC'],
                                         simulation_try['typeWBC'],
                                         naive=False)

                            if ret:
                                print('    Succeed')
                                nums_success[ii, test_id] += 1
                            else:
                                print('    Failed')
                        print("--> Tested magnitude: " + str(np.around(magnitude_try[test_id], 1)) + " [m/s] with phase " +
                              str(phase_deg) +" deg. Success rate: " + str(nums_success[ii, test_id]/10), flush=True)

                        print('#' * 60, flush=True)
                    print("Phase" + str(phase_deg) + " deg", flush=True)
                    print("Tested maginutude: \n" + str(magnitude_try[test_id]) + " [m/s]", flush=True)
                    print("Success rate: \n" + str(nums_success[:, test_id]/10), flush=True)
                    print('#' * 60, flush=True)

                print('#' * 60, flush=True)
                print("Final report. Success rates for all tested magnitudes(columns) and phases(rows)", flush=True)
                print("str(nums_success / 10)", flush=True)
                print('#' * 60, flush=True)




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
