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

    def __init__(self):
        """ Initializes a LegPos object """

        pass

    def get_leg_coords(self, keycodes, dt):
        """ Determines the next frame's leg coordinates based on controller input """

        # determine how the robot will move
        dx = 0
        dy = 0
        dz = 0
        dzrot = 0 # rotation about z axis

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
            dx *= dt*max_horiz_speed
            dy *= dt*max_horiz_speed
        else:
            dx *= dt*max_horiz_speed*np.sqrt(2.0)
            dy *= dt*max_horiz_speed*np.sqrt(2.0)

        # check for controller input regarding left/right rotation
        if ord('a') in keycodes:
            dzrot -= 1
        if ord('d') in keycodes:
            dzrot += 1

        dzrot *= dt*max_zrot_speed

        # check for controller input regarding vertical translation
        if ord('n') in keycodes:
            dz += 1
        if ord('m') in keycodes:
            dz -= 1

        dz *= dt*max_vert_speed
