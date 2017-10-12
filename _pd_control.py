import math

class PDControlLoop:

    def __init__(self, direction, distance_to_wall, Kp=10., Kd=5., angle_coefficient=1.):

        self.direction = direction                  # 1 for LHR, -1 for RHR
        self.distance_desired = distance_to_wall    # Desired distance to be away from wall
        self.Kp = Kp                                # kp Constant. default 10.
        self.Kd = Kd                                # kd Constant. default 5.
        self.angle_coefficient = angle_coefficient

        self.err_curr = 0.0                          # error: current distance - desired distance to the wall
        self.err_prev = 0.0                          # previous error
        self.err_deriv = 0.0                         # error derivative: current error - previous error
        self.c = 0.0                                 # result



    def step(self, distance_curr, angle_with_closet_obstacle):

        self.err_curr = distance_curr - self.distance_desired      # Calculate current error
        self.err_deriv = self.err_curr - self.err_prev             # Calculate error derivative


        ## Calculate angle adjustment result from PD formula
        self.c = self.direction * (self.Kp * self.err_curr + self.Kd * self.err_deriv) \
                    + self.angle_coefficient * (angle_with_closet_obstacle - math.pi * self.direction / 2)

        self.err_prev = self.err_curr                              # Update previous error


        return self.c


    def print_status(self):

        print("Error current    :"+ str(self.err_curr))
        print ("Error prev      :"+ str(self.err_prev))
        print ("Error Derivative:"+ str(self.err_deriv))
        print("Calculated angular z:    "+ str(self.c))
        print("- - - - - - - - - - - - - - - -  ")
        print()






