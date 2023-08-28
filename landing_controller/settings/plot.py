from screeninfo import get_monitors
import matplotlib.pyplot as plt
#################
# plot settings #
#################
PLOT_SETTINGS = {}
PLOT_SETTINGS['show'] = False
PLOT_SETTINGS['save'] = True

# do not modify the following
plt.ion()
width_inches = 0.
height_inches = 0.
mm2inches = 0.0393701
for m in get_monitors():
    if m.is_primary:
        width_inches = m.width_mm * mm2inches
        height_inches = m.height_mm * mm2inches
        break
PLOT_SETTINGS['width_inches'] = width_inches
PLOT_SETTINGS['height_inches'] = height_inches


PLOT_SETTINGS['directory_path'] = '' # filled at runtime if PLOT_SETTINGS['save']  is true

