import time
import curses
import numpy as np
import serial
from kinematics import Kinematics

# NOTES: 
# distances in inches, time in seconds
# for any array of size [6,3]: first index is leg index, second index is motor index 
# leg indices correspond to the following legs:
#   [0]: back left
#   [1]: middle left
#   [2]: front left
#   [3]: back right
#   [4]: middle right
#   [5]: front right
# PW stands for pulse width

class HFW(object):
    """ Main hexapod firmware class """

    frame_period = 1.0 # s

    # distances used in leg_coords_to_angles()
    # all units in inches
    z_u = 0.625 # z distance between upper leg pivot and origin
    x_s = np.array([-2.25, 0.0, 2.25, -2.25, 0.0, 2.25]) # x distance between shoulder pivot and origin
    y_s = np.array([1.125, 1.5625, 1.125, -1.125, -1.5625, -1.125]) # y distance between shoulder pivot and origin
    d_su = 0.5 # distance between shoulder pivot and upper leg pivot in plane parallel to upper and lower leg
    d_sg = 0.125 # distance between shoulder pivot and ground contact in plane normal to upper leg
    d_ul = 2.0 # upper leg length
    d_lg = 2.625 # lower leg length

    # for leg_angles_to_PWs()
    std_angles = np.array([ # reference angles at which pulse width is known (rad)
        [0.0, 0.0, 0.5*np.pi],
        [0.0, 0.0, 0.5*np.pi],
        [0.0, 0.0, 0.5*np.pi],
        [0.0, 0.0, 0.5*np.pi],
        [0.0, 0.0, 0.5*np.pi],
        [0.0, 0.0, 0.5*np.pi]])
    std_PWs = np.array([ # pulse widths at reference angles 
        [1910, 1670, 1450],
        [1685, 1640, 1470],
        [1050, 1690, 1440],
        [1130, 1390, 1600],
        [1475, 1380, 1505],
        [2125, 1420, 1420]])
    motor_m = 2000.0/np.pi # the full range of pulse widths, from 500 to 2500, corresponds to pi radians of rotation
                           # pulse width units per rad
    motor_m_sign = np.array([ # the sign of m
        [1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0]])

    def __init__(self, simulate = False):
        """ Initializes a HFW object """

        self.simulate = simulate

        # initialize curses, change terminal settings for game-like keyboard input
        self.stdscr = curses.initscr() # initialize curses
        curses.noecho() # disable terminal echo on key press
        curses.cbreak() # allow instant response to key presses
        self.stdscr.keypad(True) # enable keypad mode so you can input non alphanumeric keys
        self.stdscr.nodelay(True) # polling for keys does not halt program

        try:

            # initialize Kinematics 
            self.kinematics = Kinematics()

            # open connection to servo controller
            if not self.simulate:
                self.port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=3.0)

            # run main loop
            self.quit = False
            while not self.quit:
                self.step()

            # close connection
            if not self.simulate:
                self.port.close()

        finally:
            # reset terminal settings before exiting for any reason
            curses.nocbreak()
            self.stdscr.keypad(False)
            self.stdscr.nodelay(False)
            curses.echo()
            curses.endwin()
            print "Program exited safely"

    def step(self):
        """ Reads controller input and runs the hexapod accordingly for the next frame period """

        print "stepping\r"

        # record time at which computation starts
        start_time = time.time()

        # apply to servo controller previous frame's leg pulse widths, instruct to move over a frame period
        # assumes this frame won't take longer than a frame period to calculate
        # if the program attempted to assign invalid motor positions, quit
        if hasattr(self, "leg_PWs"):
            if not self.move_motors(self.leg_PWs):
                self.quit = True

        # read controller input
        # TODO: read keystrokes into an array or list or whatever instead of printing them out
        # do things with that list once you have it
        keycodes = set()
        while 1:
            keycode = self.stdscr.getch()
            if keycode == -1:
                break;
            else:
                keycodes.add(keycode)

        # if user pressed q, quit
        if ord('q') in keycodes:
            self.quit = True

        # pass the rest of the keycodes to the module that controls leg positions
            # calculate new floor position based on input
            # calculate new leg positions based on new floor position
        leg_coords = self.kinematics.get_leg_coords(keycodes, self.frame_period)

        # calculate new leg angles based on new leg positions
        self.leg_angles = np.zeros([6,3],dtype=float)
        for ileg in range(0,6):
            self.leg_angles[ileg] = self.leg_coords_to_angles(leg_coords[ileg], ileg)

        # calculate new leg pulse widths based on new leg angles
        self.leg_PWs = self.leg_angles_to_PWs(self.leg_angles)

        # record time at which computation ends
        end_time = time.time()

        # sleep for the remainder of the frame period
        time.sleep(self.frame_period - (end_time - start_time))

    # argument is a [6,3] array
    def move_motors(self, leg_PWs):
        """ Moves the motors according to the specified pulse widths. Returns true if successful """

        # check that all the pulse widths are valid (i.e. between 500 and 2500)
        for ileg in range(0,6):
            for imotor in range(0,3):
                if leg_PWs[ileg][imotor] < 500 or 2500 < leg_PWs[ileg][imotor]:
                    print "ERROR: leg PW of", leg_PWs[ileg][imotor], "invalid on leg", ileg, "motor", imotor
                    return False

        # format the string that we send to the servo controller

        cmd_str = "" 

        for iside in range(0,2):
            for ileg in range(0,3):
                for imotor in range(0,3):

                    motor_index = iside*16 + ileg*4 + imotor
                    cmd_str_seg = "#" + str(motor_index) + "P" + str(leg_PWs[iside*3+ileg][imotor])
                    cmd_str += cmd_str_seg

        cmd_str += "T"
        cmd_str += str(int(self.frame_period*1000))
        cmd_str += "\r"

        # send the string to the servo controller
        if not self.simulate:
            #######################self.port.write(cmd_str)
            pass

        return True

    # first argument is 3 element array: x_g, y_g, z_g
    # second argument is the leg index (an int in range(0,6))
    def leg_coords_to_angles(self, coords, ileg):
        """ Converts one set of leg coords to leg angles """

        # this function is the result of several pages of diagrams and math on paper, 
        #   hence the bad documentation
        #   wow such proprietary

        x_g = coords[0]
        y_g = coords[1]
        z_g = coords[2]

        # distance in x-y plane between shoulder pivot and ground contact
        r_sg = np.sqrt((x_g-self.x_s[ileg])**2 + (y_g-self.y_s[ileg])**2)

        # distance between upper leg pivot and ground contact in plane parallel to upper and lower leg
        d_ug = np.sqrt((r_sg-self.d_su)**2 + (self.z_u-z_g)**2)

        # shoulder angle
        theta_s = 0.0
        if (ileg < 3):
            theta_s = np.arctan2((x_g-self.x_s[ileg]),(y_g-self.y_s[ileg])) - np.arcsin(self.d_sg/r_sg)
        else:
            theta_s = np.arctan2((x_g-self.x_s[ileg]),-(y_g-self.y_s[ileg])) - np.arcsin(self.d_sg/r_sg)

        # upper leg angle
        theta_l = np.arccos((self.d_ul**2+self.d_lg**2-d_ug**2)/(2*self.d_ul*self.d_lg))

        # lower leg angle
        theta_u = np.arcsin(self.d_lg*np.sin(theta_l)/d_ug) - np.arctan2((self.z_u-z_g),(r_sg-self.d_su))

        angles = np.array([theta_s, theta_u, theta_l])
        return angles
    
    # argument is a [6,3] array
    def leg_angles_to_PWs(self, angles):
        """ Converts all leg angles to motor pulse widths """

        #formula for pulse widths as a function of angle:
        # (target PW) = ((target angle) - (angle for known PW))*m + (known PW)
        #       where m = +/- 2000/pi units of pulse width per radian (sign depends on motor rotation direction)
        return ((angles - self.std_angles)*self.motor_m*self.motor_m_sign).astype(int) + self.std_PWs        


