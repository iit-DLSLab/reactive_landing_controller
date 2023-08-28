import matplotlib.pyplot as plt
import numpy as np
import datetime
from base_controllers.utils.common_functions import *
DEG2RAD = np.pi/180

###############################
# TIMINGS DESCRIPTION CLASSES #
###############################

class Event:
    def __init__(self, name):
        self.name = name
        self.t = 0.
        self.sample = -1
        self.detected = False

    def set(self, t, sample):
        self.t = t
        self.sample = sample
        self.detected = True

    def __repr__(self):
        return f"(t: \t{self.t} [s], sample: \t{self.sample}, detected: \t{self.detected})"

class LcEvents:
    def __init__(self):
        self.lift_off = Event('Lift off')
        self.apex = Event('Apex')
        self.touch_down = Event('Touch down')


        self._list_of_events = [self.lift_off, self.apex, self.touch_down]
        self._headers = ['Event', 'Time (s)', '# Sample']

    def __repr__(self):
        return f"Landing controller events \n  \t lift off: \t\t {self.lift_off} \n  \t apex: \t\t {self.apex} \n   \t touch down: \t {self.touch_down} \n"


#########################
# normal var descriptor #
#########################
class Normal:
    def __init__(self, mean, std, size):
        self.mean = mean
        self.std = std
        self.size = size

    def draw(self):
        return np.random.normal(self.mean, self.std, self.size)



def initCond2str(simulation, printVideo=True, speedUpDown=1.):
    SIMstr = ''
    SIMstr += 'id: '   + simulation['id'] + '\n'
    SIMstr += 'name: ' + simulation['name'] + '\n'
    SIMstr += 'base height: ' + str(np.round(simulation['pose'][2], 3)) + ' m\n'
    SIMstr += 'base orientation: ' + str(np.round(simulation['pose'][3:]/DEG2RAD, 3)) + ' deg\n'
    SIMstr += 'base lin vel: ' + str(np.round(simulation['twist'][:3], 3)) + ' m/s\n'
    SIMstr += 'base ang vel: ' + str(np.round(simulation['twist'][3:]/DEG2RAD, 3)) + ' deg/s\n'
    SIMstr += 'useWBC: ' +  str(simulation['useWBC']) + '\n'
    SIMstr += 'useIK: ' + str(simulation['useIK'])
    return SIMstr

def findLimitsInitCond2str_angularTest(simulation):
    s0 = ' FIND LIMITS \n'
    s0 += '\n'
    s0 += ' base height: ' + str(np.round(simulation['pose'][2], 3)) + ' m \n'
    s0 += ' base lin vel: ' + str(np.round(simulation['twist'][:3], 3)) + ' m/s \n'
    s0 += ' test limit: ' + simulation['test_limit'] +'\n'
    s0 += ' useWBC: ' + str(simulation['useWBC']) + ' \n'
    s0 += ' useIK: ' + str(simulation['useIK']) + ' '

    max_row = max(len(row) for row in s0.split('\n'))

    s1 = ''
    for row in s0.split('\n'):
        l = len(row)
        row = '*' + row
        if l == 0:
            row += '*' * max_row
        elif 0<l < max_row:
            row += ' ' * (max_row - l)
        row += '*\n'
        s1 += row
    s1 = s1[:-1] # remove the last \n
    n_char = max(len(row) for row in s1.split('\n'))
    sim_str = '*'*n_char + '\n' +  s1 + '\n' + '*'*n_char
    return sim_str

def findLimitsInitCond2str(simulation):
    s0 = ' FIND LIMITS \n'
    s0 += '\n'
    s0 += ' base height: ' + str(np.round(simulation['pose'][2], 3)) + ' m \n'
    s0 += ' base orientation: ' + str(np.round(simulation['pose'][3:]/DEG2RAD, 3)) + ' deg \n'
    s0 += ' base ang vel: ' + str(np.round(simulation['twist'][3:]/DEG2RAD, 3)) + ' deg/s \n'
    s0 += ' useWBC: ' + str(simulation['useWBC']) + ' \n'
    s0 += ' useIK: ' + str(simulation['useIK']) + ' '

    max_row = max(len(row) for row in s0.split('\n'))

    s1 = ''
    for row in s0.split('\n'):
        l = len(row)
        row = '*' + row
        if l == 0:
            row += '*' * max_row
        elif 0<l < max_row:
            row += ' ' * (max_row - l)
        row += '*\n'
        s1 += row
    s1 = s1[:-1] # remove the last \n
    n_char = max(len(row) for row in s1.split('\n'))
    sim_str = '*'*n_char + '\n' +  s1 + '\n' + '*'*n_char
    return sim_str


def manipulateFig(fig, filename, PLOT_SETTINGS, directory,  verbose = False):
    fig.set_size_inches([PLOT_SETTINGS['width_inches'], PLOT_SETTINGS['height_inches']])

    if PLOT_SETTINGS['save']:
        plt.savefig(directory + '/' + filename+ '.pdf')
        plt.close()
        del fig
        if verbose:
            print('Plot ' + filename+ '.pdf saved', flush=True)

    if PLOT_SETTINGS['show']:
        plt.show()


def makePlots(p, figures, PLOT_SETTINGS, directory, start=0, end=-1, verbose = False):
    id = 0
    for name in figures:
        if name == 'q' and figures[name]:
            # joint position
            fig = plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, sharex=True, sharey=False,
                  start=start, end=end)
            manipulateFig(fig, 'q', PLOT_SETTINGS, directory, verbose)
            id += 1

        elif name == 'qd' and figures[name]:
            # joint velocity
            fig = plotJoint('velocity', time_log=p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log, sharex=True,
                  sharey=False, start=start, end=end)
            manipulateFig(fig, 'qd', PLOT_SETTINGS, directory, verbose)
            id += 1

        elif name == 'tau' and figures[name]:
            # joint torques
            fig = plotJoint('torque', time_log=p.time_log, tau_log=p.tau_log, tau_ffwd_log=p.tau_ffwd_log,
                  tau_des_log=p.tau_fb_log, sharex=True, sharey=False, start=start, end=end)
            manipulateFig(fig, 'tau', PLOT_SETTINGS, directory, verbose)
            id += 1

        elif name == 'com' and figures[name]:
            # com position
            fig = plotFrame('position', time_log=p.time_log, des_Pose_log=p.comPoseW_des_log, Pose_log=p.comPoseW_log,
                  title='CoM', frame='W', sharex=True, sharey=False, start=start, end=end)
            manipulateFig(fig, 'com', PLOT_SETTINGS, directory, verbose)
            id += 1

        elif name == 'vcom' and figures[name]:
            # com velocity
            fig = plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.comTwistW_des_log, Twist_log=p.comTwistW_log,
                  title='CoM', frame='W', sharex=True, sharey=False, start=start, end=end)
            manipulateFig(fig, 'vcom', PLOT_SETTINGS, directory, verbose)
            id += 1

        elif name == 'w_contacts' and figures[name]:
            # feet position in w-frame and contact flag
            fig =  plotContacts('position', time_log=p.time_log, des_LinPose_log=p.W_contacts_des_log,
                     LinPose_log=p.W_contacts_log,
                     contact_states=p.contact_state_log, frame='W', sharex=True, sharey=False, start=start, end=end)
            manipulateFig(fig, 'W_feet', PLOT_SETTINGS, directory, verbose)
            id += 1

        elif name == 'b_contacts' and figures[name]:
            # feet position in b-frame and contact flag
            fig = plotContacts('position', time_log=p.time_log, des_LinPose_log=p.B_contacts_des_log,
                     LinPose_log=p.B_contacts_log,
                     contact_states=p.contact_state_log, frame='B', sharex=True, sharey=False, start=start, end=end)
            manipulateFig(fig, 'B_feet', PLOT_SETTINGS, directory, verbose)
            id += 1

        elif name == 'grfs_contacts' and figures[name]:
            # force in world
            fig = plotContacts('GRFs', time_log=p.time_log, des_Forces_log=p.grForcesW_des_log,
                     Forces_log=p.grForcesW_log, contact_states=p.contact_state_log, frame='W',
                     sharex=True, sharey=False, start=start, end=end)
            manipulateFig(fig, 'grfs', PLOT_SETTINGS, directory, verbose)
            id += 1

        else:
            print('Cannot understand ' + name)

def setId(simulation, sim_counter):
    if sim_counter < 10:
        simulation['id'] = '0' + str(sim_counter)
    else:
        simulation['id'] = str(sim_counter)

def saveAllInitConds(directory, SIMS, speedUpDown=1., verbose=False):
    f = open(directory + "/ALL_simulations.txt", "w")
    for simulation in SIMS:
        f.write(initCond2str(simulation, speedUpDown=speedUpDown) + '\n' + '-' * 10 + '\n')
    f.close()
    if verbose:
        print(directory + '/ALL_simulations.txt saved')

def saveInitConds(directory, simulation, speedUpDown=1., verbose=False):
    f = open(directory + "/simulations.txt", "w")
    f.write(initCond2str(simulation, speedUpDown=speedUpDown) + '\n' + '-' * 10 + '\n')
    f.close()
    if verbose:
        print(directory + '/simulations.txt saved')

def setSavePath(SETTINGS, directory='simulation'):
    #if SETTINGS['WORKSPACE']['save'] or SETTINGS['PLOTS']['save'] or SETTINGS['save_log'] or SETTINGS['INIT_CONDS']['save_all']: # set save path in all the cases
        now = datetime.datetime.now()
        now_s = str(now)
        now_s = now_s.replace('-', '')
        now_s = now_s.replace(' ', '_')
        now_s = now_s.replace(':', '')
        now_s = now_s[: now_s.find('.')]
        try:
            os.mkdir(os.environ['LOCOSIM_DIR'] + '/landing_controller/'+directory)
        except OSError as error:
            pass
        SETTINGS['save_path'] = os.environ['LOCOSIM_DIR'] + '/landing_controller/' + directory +"/"+ now_s
        os.mkdir(SETTINGS['save_path'])
