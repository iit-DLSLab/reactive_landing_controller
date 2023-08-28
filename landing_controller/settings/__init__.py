from .init_conds import INIT_CONDS
from .fig_list import FIG_LIST
from .plot import PLOT_SETTINGS
from .simulation import SIMS_SETTINGS
from .video import VIDEO
from .workspace import WORKSPACE_SETTINGS

SETTINGS = {}
SETTINGS['WORKSPACE'] = WORKSPACE_SETTINGS
SETTINGS['FIGURES'] = FIG_LIST
SETTINGS['PLOTS'] = PLOT_SETTINGS
SETTINGS['SIMS'] = SIMS_SETTINGS
SETTINGS['VIDEO'] = VIDEO
SETTINGS['INIT_CONDS'] = INIT_CONDS

SETTINGS['verbose'] = False
SETTINGS['save_log'] = True
SETTINGS['save_path'] = ''
SETTINGS['finally_close'] = False


# modificable settings (all the settings can be modified, but it's very likely that you will not need them)
# simulation settins are in simulation.py

# plot
SETTINGS['PLOTS'] ['show'] = False
SETTINGS['PLOTS'] ['save'] = True

# video
SETTINGS['VIDEO']['save'] = True

# workspace
SETTINGS['WORKSPACE']['save'] = False
