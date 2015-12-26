#!/usr/bin/python

import sys
from hexapod_firmware import HFW

simulate = False
graphic_debug = False

# parse arguments

# -s = run in simulation mode (no communication port is opened)
if '-s' in sys.argv:
    simulate = True

# -g = run with graphical debugging
if '-g' in sys.argv:
    graphic_debug = True

# run the program
hfw = HFW(simulate, graphic_debug)

