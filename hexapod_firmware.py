import time
import numpy as np

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


    def __init__(self):
        """ Initializes a HFW object """


        self.quit = False
        while self.quit == False:
            self.step()

    def step(self):
        """ Reads controller input and runs the hexapod accordingly for the next frame period """

        print "stepping"

        # record time at which computation starts
        start_time = time.time()

        # apply to servo controller previous frame's leg pulse widths, instruct to move over a frame period
        # assumes this frame won't take loner than a frame period to calculate

        # read controller input

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
        for i in range(0,6):
            leg_angles[i] = self.leg_coords_to_angles(leg_coords[i])

        # calculate new leg pulse widths based on new leg angles

        # record time at which computation ends
        end_time = time.time()

        # sleep for the remainder of the frame period
        time.sleep(self.frame_period/1000.0 - (end_time - start_time))

    #argument is 3 element array: x_g, y_g, z_g
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

        pass


