import numpy as np
import sys, pygame
            
# distances in inches, time in seconds
# coordinate axes:
#   +z: up
#   +x: forward
#   +y: left
# rotation about vertical axis: CCW as you look down on robot (right hand rule)

# CONTROLS:
# IJKL: move forward/backward, strafe
# WASD: rotate left/right, look up/down (W and S currently not implemented)
# N: lower body
# M: raise body

# NOTES:
# Coordinates are defined with respect to the body. 
# When the controller instructs to move or rotate the robot in a particular direction or angle w.r.t. the floor,
#   we imagine the floor moving or rotating in the opposite direction or angle

class Kinematics(object):
    """ Class that reads keyboard input to move the floor and then decides leg positions accordingly """

    def __init__(self, graphic_debug = False):
        """ Initializes a Kinematics object with initial leg positions """

        # Whether or not to start a pygame screen to display leg positions
        self.graphic_debug = graphic_debug

        if self.graphic_debug:

            # init graphics
            pygame.init()

            # define screen size and colors that might be used
            self.screen_size = width, height = 800, 800
            self.BLACK = (  0,   0,   0)
            self.WHITE = (255, 255, 255)
            self.BLUE =  (  0,   0, 255)
            self.GREEN = (  0, 255,   0)
            self.RED =   (255,   0,   0)

            # initiate screen
            self.screen = pygame.display.set_mode(self.screen_size)

            # Set scaling, i.e. the number of pixels corresponding to 1 inch
            self.draw_scale = 50 # p/in

            # Set radius of dots used to represent leg positions 
            self.dot_radius = 5 # p

        self.z_floor = -1.375 # in # current z position of the floor w.r.t. the body (origin)

        # initialize starting leg coords (each row is [x,y,z])
        # leg motion will be somewhat centered around these coords
        self.starting_leg_coords = np.array([
            [-4.325,3.575,self.z_floor],   # back left  
            [0.0,5.1,self.z_floor],        # middle left  
            [4.325,3.575,self.z_floor],    # front left   
            [-4.325,-3.575,self.z_floor],  # back right  
            [0.0,-5.1,self.z_floor],       # middle right  
            [4.325,-3.575,self.z_floor]])  # front right
        self.leg_coords = np.copy(self.starting_leg_coords)

        # whether or not the legs are on the ground
        self.is_on_ground = np.ones(6, dtype = np.bool)

        # maximum speeds
        self.max_horiz_speed = 1.00 # in/s
        self.max_vert_speed = self.max_horiz_speed # in/s

        # We want the max z rotation speed to be such that the maximum speed of any of the starting leg coords
        #   w.r.t. the floor is the same as when the robot is translating
        max_xy_dist_starting_leg_coords = 0.0
        for ileg in range(0,6):
            max_xy_dist_starting_leg_coords = max(
                    max_xy_dist_starting_leg_coords, 
                    np.sqrt(self.starting_leg_coords[ileg][0]**2 + self.starting_leg_coords[ileg][1]**2))
        self.max_zrot_speed = self.max_horiz_speed / max_xy_dist_starting_leg_coords # rad/s

        self.dz_off_ground = 0.125 # in # distance off the ground a foot is when that foot is not on the ground

        # Define "nominal distance" to be the distance traveled by the body of the robot as if it were
        #   translating steadily in 1 direction.
        # Nominal distance represents the distance traveled by the starting leg coords averaged over all legs.
        #   It should be approximately the same for any combination of translation and rotation.
        # Because the robot's speed may vary, the motion of the legs is defined in terms of nominal distance,
        #   not time.

        # Define the pattern with which the robot picks up and puts down its legs.
        # First, we define the number of legs that are off the ground at any given time during steady operation
        # NOTE: MUST be 1, 2, or 3
        self.n_legs_off_ground = 3 
        # Next, we define how many legs are picked up each time the robot travels a nominal distance D0
        # NOTE: MUST be a nonzero number that self.n_legs_off_ground is divisible by
        #   i.e. the robot picks up one leg at a time or (self.n_legs_off_ground) legs at once
        self.n_legs_pickup_at_once = 3 
        # order to pick up legs when walking 
        self.pickup_order = np.array([1,3,5,0,2,4])  
        # iterator on the above array
        self.last_picked_up = 5 

        # Whenever the robot travels a certain nominal distance, one or more legs pick up. 
        #   That nominal distance interval D0 is defined as:
        self.nom_dist_between_pickups = 0.4 * float(self.n_legs_pickup_at_once) # in 

        # Nominal distance traveled since last leg pickup:
        self.nom_dist_traveled = self.nom_dist_between_pickups # in

        # A leg spends a nominal distance DA in the air:
        self.nom_dist_air = self.nom_dist_between_pickups * float(self.n_legs_off_ground)/float(self.n_legs_pickup_at_once)
        # and a nominal distance DG on the ground:
        self.nom_dist_ground = self.nom_dist_between_pickups * float(6 - self.n_legs_off_ground)/float(self.n_legs_pickup_at_once)
        # When it's in the air, it's moving toward its target placement point (TPP) 
        # The TPP is defined as the point at which that leg, starting at its starting leg coord,
        #   would have moved to in a nominal distance DG/2,
        #   given the current translation and rotation of the robot.
        # The leg moves toward the TPP at a rate that would cause it to reach the TPP after 
        #   the robot has traveled a nominal distance DG with that leg up.
        # The caveat to this is that sudden changes in velocity or rotation rate could drastically
        #   change the TPP.
        # Therefore, we enforce a maximum speed to the leg's motion while in the air:
        self.max_leg_speed = 4.0*self.max_horiz_speed # in/s 

    def get_leg_coords(self, keycodes, dt):
        """ Determines the next frame's leg coordinates based on controller input """

        if self.graphic_debug:

            # Fill background
            self.screen.fill(self.WHITE)

            # Draw coordinate axes
            pygame.draw.line(self.screen, self.BLACK, 
                    [self.screen_size[0]/2, 0], 
                    [self.screen_size[0]/2, self.screen_size[1]], 1)
            pygame.draw.line(self.screen, self.BLACK, 
                    [0, self.screen_size[1]/2], 
                    [self.screen_size[0], self.screen_size[1]/2], 1)

        ############### DETERMINE THIS STEP'S MOTION BASED ON CONTROLLER INPUT #################################
            
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

        ############### MOVE THE LEGS BASED ON THIS STEP'S MOTION #############################################

        # increment the nominal distance traveled since last leg pickup
        self.nom_dist_traveled_this_step = 0.0
        if dx != 0 or dy != 0 or dzrot != 0:
            self.nom_dist_traveled_this_step = dt*self.max_horiz_speed
            self.nom_dist_traveled += dt*self.max_horiz_speed

        # determine if any legs need to be put down or picked up
        # then reset the nominal distance traveled since last pickup
        if self.nom_dist_traveled >= self.nom_dist_between_pickups:

            # increment in the ring of integers mod 6
            self.last_picked_up = (self.last_picked_up + self.n_legs_pickup_at_once) % 6  
            self.nom_dist_traveled -= self.nom_dist_between_pickups
            
            # put down the leg(s) that was(were) picked up the longest time ago
            for ileg in range(0,self.n_legs_pickup_at_once):
                self.is_on_ground[self.pickup_order[(self.last_picked_up - self.n_legs_off_ground - ileg)%6]] = True 

            # we pick up the next leg on the next step, rather than this step, 
            #   so we don't have a small amount of time with only 2 legs on the ground
            # do that here:
        else:
            for ileg in range(0,self.n_legs_pickup_at_once):
                self.is_on_ground[self.pickup_order[(self.last_picked_up - ileg)%6]] = False

        # move the legs
        for ileg in range(0,6):

            # move the legs that are on the ground
            if self.is_on_ground[ileg]:

                # translate
                self.leg_coords[ileg] += np.array([dx, dy, dz])

                # rotate
                self.leg_coords[ileg] = np.array([
                    self.leg_coords[ileg][0]*np.cos(dzrot) - self.leg_coords[ileg][1]*np.sin(dzrot),
                    self.leg_coords[ileg][0]*np.sin(dzrot) + self.leg_coords[ileg][1]*np.cos(dzrot),
                    self.leg_coords[ileg][2]])

            # move the legs that are off the ground
            else:

                # calculate the TPP
                TPP = np.copy(self.starting_leg_coords[ileg])
                #   translate
                if dx != 0.0 or dy != 0.0: # check to prevent divide-by-zero
                    TPP += np.array([
                        -dx/np.sqrt(dx**2+dy**2)*self.nom_dist_ground/2.0, 
                        -dy/np.sqrt(dx**2+dy**2)*self.nom_dist_ground/2.0, 
                        0.0])
                # rotate
                if dzrot != 0.0: # check to avoid extra number crunching
                    angle_to_rotate = self.nom_dist_ground/2.0 * self.max_zrot_speed / self.max_horiz_speed
                    TPP = np.array([
                        TPP[0]*np.cos(angle_to_rotate) - TPP[1]*np.sin(angle_to_rotate),
                        TPP[0]*np.sin(angle_to_rotate) + TPP[1]*np.cos(angle_to_rotate),
                        TPP[2]])

                # The leg travels in a line toward the TPP. 
                # That line is described by the following displacement vector:
                displacement_to_TPP = TPP - self.leg_coords[ileg]

                # Calculate the remaining nominal distance the robot must travel for this leg to reach the TPP
                for j in range(0, self.n_legs_off_ground):
                    if ileg == self.pickup_order[(self.last_picked_up - (self.n_legs_off_ground-1-j)) % 6]:
                        nom_dist_to_TPP = float(j/self.n_legs_pickup_at_once + 1)*self.nom_dist_between_pickups - (self.nom_dist_traveled - self.nom_dist_traveled_this_step)
                        break

                # Calculate the displacement vector for this step
                displacement_this_step = displacement_to_TPP * self.nom_dist_traveled_this_step / nom_dist_to_TPP

                # Enforce the maximum speed on legs that are not on the ground
                dist_this_step = np.sqrt(displacement_this_step[0]**2 + displacement_this_step[1]**2)
                dist_this_step_max = dt*self.max_leg_speed
                if dist_this_step > dist_this_step_max:
                    displacement_this_step *= dist_this_step_max / dist_this_step

                # Apply this step's displacement to this leg
                self.leg_coords[ileg] += np.copy(displacement_this_step)
                
                if self.graphic_debug:

                    # draw the TPP
                    pygame.draw.circle(self.screen, self.GREEN,
                            [
                                self.screen_size[0]/2 + int(self.draw_scale*TPP[0]),
                                self.screen_size[1]/2 + int(self.draw_scale*TPP[1])
                                ], self.dot_radius+3)

                    # draw a line from the current leg pos to the TPP
                    pygame.draw.line(self.screen, self.GREEN,
                            [
                                self.screen_size[0]/2 + int(self.draw_scale*self.leg_coords[ileg][0]),
                                self.screen_size[1]/2 + int(self.draw_scale*self.leg_coords[ileg][1])
                                ],
                            [
                                self.screen_size[0]/2 + int(self.draw_scale*TPP[0]),
                                self.screen_size[1]/2 + int(self.draw_scale*TPP[1])
                                ], 3)

        if self.graphic_debug:

            for ileg in range(0,6):

                # Draw leg positions
                # Blue indicates leg is on ground, red indicates leg is in air.
                if self.is_on_ground[ileg]:
                    leg_color = self.BLUE
                else:
                    leg_color = self.RED
                pygame.draw.circle(self.screen, leg_color, 
                        [
                            self.screen_size[0]/2 + int(self.draw_scale*self.leg_coords[ileg][0]), 
                            self.screen_size[1]/2 + int(self.draw_scale*self.leg_coords[ileg][1])
                            ], self.dot_radius)

                # draw the starting leg coords
                pygame.draw.circle(self.screen, self.BLACK,
                        [
                            self.screen_size[0]/2 + int(self.draw_scale*self.starting_leg_coords[ileg][0]), 
                            self.screen_size[1]/2 + int(self.draw_scale*self.starting_leg_coords[ileg][1])
                            ], self.dot_radius+2, 1)


            # refresh the screen
            pygame.display.flip()

        return self.leg_coords
