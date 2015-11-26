import numpy as np
            
# distances in inches, time in seconds
# coordinate axes:
#   +z: up
#   +x: forward
#   +y: left
# rotation about vertical axis: CCW as you look down on robot (right hand rule)

# CONTROLS:
# IJKL: move forward/backward, strafe
# WASD: rotate left/right, look up/down (W,S currently not implemented)
# N: lower body
# M: raise body
# backspace: toggle in and out of rest position (currently not implemented)

# NOTES:
# Coordinates are defined with respect to the body. 
# When the controller instructs to move or rotate the robot in a particular direction or angle w.r.t. the floor,
#   we imagine the floor moving or rotating in the opposite direction or angle

class Kinematics(object):
    """ Class that reads keyboard input to move the floor and then decides leg positions accordingly """

    max_horiz_speed = 0.25 # in/s
    max_vert_speed = 0.25 # in/s
    max_zrot_speed = np.pi/8.0 # rad/s # maximum angular speed of rotation about z axis

    z_floor = -1.375 # in

    def __init__(self):
        """ Initializes a Kinematics object with initial leg positions """

        # initialize starting leg coords (each row is [x,y,z])
        self.leg_coords = np.array([
            [-3.625,2.875,self.z_floor],   # back left  
            [0.0,4.1,self.z_floor],        # middle left  
            [3.625,2.875,self.z_floor],    # front left   
            [-3.625,-2.875,self.z_floor],  # back right  
            [0.0,-4.1,self.z_floor],       # middle right  
            [3.625,-2.875,self.z_floor]])  # front right

        # whether or not the legs are on the ground
        self.is_on_ground = np.ones(6, dtype = np.bool)

    def get_leg_coords(self, keycodes, dt):
        """ Determines the next frame's leg coordinates based on controller input """

        # determine how the floor will move w.r.t. the robot
        dx = 0 # in
        dy = 0 # in
        dz = 0 # in
        dzrot = 0 # rotation about z axis (rad)

        # check for controller input regarding horizontal translation
        if ord('i') in keycodes:
            dx -= 1
        if ord('k') in keycodes:
            dx += 1
        if ord('j') in keycodes:
            dy -= 1
        if ord('l') in keycodes:
            dy += 1

        if dx == 0 or dy == 0:
            dx *= dt*self.max_horiz_speed
            dy *= dt*self.max_horiz_speed
        else:
            dx *= dt*self.max_horiz_speed*np.sqrt(2.0)
            dy *= dt*self.max_horiz_speed*np.sqrt(2.0)

        # check for controller input regarding left/right rotation
        if ord('a') in keycodes:
            dzrot -= 1
        if ord('d') in keycodes:
            dzrot += 1

        dzrot *= dt*self.max_zrot_speed

        # check for controller input regarding vertical translation
        if ord('n') in keycodes:
            dz += 1
        if ord('m') in keycodes:
            dz -= 1

        dz *= dt*self.max_vert_speed

        # move the legs that are on the ground
        for ileg in range(0,6):
            if self.is_on_ground[ileg]:

                # translate
                self.leg_coords[ileg] += np.array([dx, dy, dz])

                # rotate
                self.leg_coords[ileg] = np.array([
                    self.leg_coords[ileg][0]*np.cos(dzrot) - self.leg_coords[ileg][1]*np.sin(dzrot),
                    self.leg_coords[ileg][0]*np.sin(dzrot) + self.leg_coords[ileg][1]*np.cos(dzrot),
                    self.leg_coords[ileg][2]])

        return self.leg_coords
