import time
import curses
import numpy as np
import serial

# NOTES: 
# for any array of size [6,3]: first index is leg index, second index is motor index 
# PW stands for pulse width

class HFW(object):
    """ Main hexapod firmware class """

    frame_period = 1000.0 # ms

    # for leg_coords_to_angles()
    s = 1.0 # side index (+1 for +y side, -1 for -y side)
    z_u = 0.625 # z distance between upper leg pivot and origin
    x_s = 2.25 # x distance between shoulder pivot and origin
    y_s = 1.125 # y distance between shoulder pivot and origin
    d_su = 0.5 # distance between shoulder pivot and upper leg pivot in plane parallel to upper and lower leg
    d_sg = 0.125 # distance between shoulder pivot and ground contact in plane normal to upper leg
    d_ul = 2.0 # upper leg length
    d_lg = 2.625 # lower leg length

    # for leg_angles_to_PWs()
    std_angles = np.array([
        [0.0, 0.0, 90.0],
        [0.0, 0.0, 90.0],
        [0.0, 0.0, 90.0],
        [0.0, 0.0, 90.0],
        [0.0, 0.0, 90.0],
        [0.0, 0.0, 90.0]])
    std_PWs = np.array([
        [1910, 1670, 1450],
        [1685, 1640, 1470],
        [1050, 1690, 1440],
        [1130, 1390, 1600],
        [1475, 1380, 1505],
        [2125, 1420, 1420]])
    motor_m = 2000.0/180.0
    motor_m_sign = np.array([
        [1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0]])

    def __init__(self):
        """ Initializes a HFW object """

        # initialize curses, change terminal settings for game-like keyboard input
        self.stdscr = curses.initscr() # initialize curses
        curses.noecho() # disable terminal echo on key press
        curses.cbreak() # allow instant response to key presses
        self.stdscr.keypad(True) # enable keypad mode so you can input non alphanumeric keys
        self.stdscr.nodelay(True) # polling for keys does not halt program

        try:

            # open connection to servo controller
            self.port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=3.0)

            # run main loop
            self.quit = False
            while not self.quit:
                self.step()

            # close connection
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
        if hasattr(self, "leg_PWs"):
            self.move_motors(self.leg_PWs)

        # read controller input
        # TODO: read keystrokes into an array or list or whatever instead of printing them out
        # do things with that list once you have it
        while 1:
            c = self.stdscr.getch()
            print c, "\r"
            if c == -1:
                break;
            elif c == ord('q'):
                self.quit = True

        # calculate new floor position based on input

        # calculate new leg positions based on new floor position
        leg_coords = np.array([     # TEMP
            [3.625,2.875,-1.375],   #
            [3.625,2.875,-1.375],   #
            [3.625,2.875,-1.375],   #
            [3.625,2.875,-1.375],   #
            [3.625,2.875,-1.375],   #
            [3.625,2.875,-1.375]])  # TEMP

        # calculate new leg angles based on new leg positions
        leg_angles = np.zeros([6,3],dtype=float)
        for ileg in range(0,6):
            leg_angles[ileg] = self.leg_coords_to_angles(leg_coords[ileg])

        # calculate new leg pulse widths based on new leg angles
        self.leg_PWs = self.leg_angles_to_PWs(leg_angles)

        # record time at which computation ends
        end_time = time.time()

        # sleep for the remainder of the frame period
        time.sleep(self.frame_period/1000.0 - (end_time - start_time))

    # argument is a [6,3] array
    def move_motors(self, leg_PWs):
        """ Moves the motors according to the specified pulse widths """

        cmd_str = "" 

        for iside in range(0,2):
            for ileg in range(0,3):
                for imotor in range(0,3):

                    motor_index = iside*16 + ileg*4 + imotor
                    cmd_str_seg = "#" + str(motor_index) + "P" + str(leg_PWs[iside*3+ileg][imotor])
                    cmd_str += cmd_str_seg

        cmd_str += "T"
        cmd_str += str(int(self.frame_period))
        cmd_str += "\r"
        #######################self.port.write(cmd_str)


    # argument is 3 element array: x_g, y_g, z_g
    def leg_coords_to_angles(self, coords):
        """ Converts one set of leg coords to leg angles """

        x_g = coords[0]
        y_g = coords[1]
        z_g = coords[2]
        r_sg = np.sqrt((x_g-self.x_s)**2 + (y_g-self.y_s)**2)
        d_ug = np.sqrt((r_sg-self.d_su)**2 + (self.z_u-z_g)**2)

        theta_s = np.arctan2((x_g-self.x_s),(y_g-self.y_s)) - np.arcsin(self.d_sg/r_sg)
        theta_l = np.arccos((self.d_ul**2+self.d_lg**2-d_ug**2)/(2*self.d_ul*self.d_lg))
        theta_u = np.arcsin(self.d_lg*np.sin(theta_l)/d_ug) - np.arctan2((self.z_u-z_g),(r_sg-self.d_su))

        angles = np.array([theta_s, theta_u, theta_l])
        return angles
    
    # argument is a [6,3] array
    def leg_angles_to_PWs(self, angles):
        """ Converts all leg angles to motor pulse widths """

        #formula for pulse widths as a function of angle:
        # (target PW) = ((target angle) - (angle for known PW))*m + (known PW)
        #       where m = +/- 2000/180 units of pulse width per degree (sign depends on motor rotation direction)
        return ((angles - self.std_angles)*self.motor_m*self.motor_m_sign).astype(int) + self.std_PWs        


