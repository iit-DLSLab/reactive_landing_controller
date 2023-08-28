from base_controllers.quadruped_controller import Controller
from landing_controller.controller.landingManager import LandingManager
from landing_controller.settings import SETTINGS # simumation details are in SETTINGS['SIMS']
from landing_controller.controller.utility import *

ROBOT_NAME = 'go1'  # go1, solo, (aliengo)
world_name = 'slow.world' # if camera is activated , world will become slow
use_gui = True

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
                                           'pid_discrete_implementation:=false'])

        p.startupProcedure()

        lm = LandingManager(p, SETTINGS)
        sim_counter = -1
        if SETTINGS['save_log']:
            sys.stdout = logfile

        for simulation in SETTINGS['SIMS']:
            if 'find_limits' in simulation['name']:
                sys.stdout = stdout
                assert simulation['name']=='find_limits', "You should use lc_find_limits.py. Exit..."

            else:
                sim_counter += 1
                setId(simulation, sim_counter)

                if SETTINGS['verbose']:
                    print("Starting simulation " + simulation['id'] + ': ' + simulation['name'])
                ret = lm.run(sim_counter,
                       simulation['pose'],
                       simulation['twist'],
                       simulation['useIK'],
                       simulation['useWBC'],
                       simulation['typeWBC'],
                       naive=False)
                if SETTINGS['verbose']:
                    print("--> simulation " + simulation['id'] + 'completed. Stabilized?' + str(ret))

                p.pid.setPDjoints(p.kp_j, p.kd_j, p.ki_j)
                p.gravityCompensation()



    except (ros.ROSInterruptException, ros.service.ServiceException) as e:
        if SETTINGS['save_log']:
            logfile.close()
            sys.stdout = stdout
        print(e)
    finally:
        if SETTINGS['save_log']:
            logfile.close()
            sys.stdout = stdout

        if SETTINGS['finally_close']:
            ros.signal_shutdown("killed")
            p.deregister_node()
            os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

        # store all the init conds
        if SETTINGS['INIT_CONDS']['save_all']:
            saveAllInitConds(SETTINGS['save_path'], SETTINGS['SIMS'], SETTINGS['VIDEO']['speedUpDown'], SETTINGS['verbose'])
            
        # these plots can be sistematically generated by makePlots() in controller/utility
        # reported here for post generation only
        # copy-past on the python console
        
        # joint position
        fig = plotJoint('position', time_log=p.time_log,
                        q_log=p.q_log, q_des_log=p.q_des_log,
                        sharex=True, sharey=False)
        # joint velocity
        fig = plotJoint('velocity', time_log=p.time_log,
                        qd_log=p.qd_log, qd_des_log=p.qd_des_log,
                        sharex=True, sharey=False)
        # joint torques
        fig = plotJoint('torque', time_log=p.time_log,
                        tau_log=p.tau_log, tau_ffwd_log=p.tau_ffwd_log, tau_des_log=p.tau_fb_log,
                        sharex=True, sharey=False)
        # com position
        fig = plotFrame('position', time_log=p.time_log,
                        des_Pose_log=p.comPoseW_des_log, Pose_log=p.comPoseW_log,
                        title='CoM', frame='W',
                        sharex=True, sharey=False)
        # com velocity
        fig = plotFrame('velocity', time_log=p.time_log,
                        des_Twist_log=p.comTwistW_des_log, Twist_log=p.comTwistW_log,
                        title='CoM', frame='W',
                        sharex=True, sharey=False)
        # feet position in w-frame and contact flag
        fig = plotContacts('position', time_log=p.time_log,
                           des_LinPose_log=p.W_contacts_des_log, LinPose_log=p.W_contacts_log, contact_states=p.contact_state_log,
                           frame='W',
                           sharex=True, sharey=False)
        # feet position in b-frame and contact flag
        fig = plotContacts('position', time_log=p.time_log,
                           des_LinPose_log=p.B_contacts_des_log, LinPose_log=p.B_contacts_log, contact_states=p.contact_state_log,
                           frame='B',
                           sharex=True, sharey=False)
        # force in world
        fig = plotContacts('GRFs', time_log=p.time_log,
                           des_Forces_log=p.grForcesW_des_log, Forces_log=p.grForcesW_log, contact_states=p.contact_state_log,
                           frame='W',
                           sharex=True, sharey=False)
