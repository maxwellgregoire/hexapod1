#!/usr/bin/python

import sys
from hexapod_firmware import HFW

simulate = False

# if first argument was "-s", we are running in simulation mode 
# (no communication port is opened)
if len(sys.argv) > 1:
    if sys.argv[1] == "-s":
        simulate = True

# run the program
hfw = HFW(simulate)

