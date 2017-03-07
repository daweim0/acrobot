'''
Created on Jul 16, 2014

@author: David Michelman
'''

# test

import math
import time


class acro_math(object):
    '''
    based on "Angular momentum Based Controller for Balancing an Inverted Double Pendulum" 
    by Mortzea Azad* and Roy Featherstone*
    
    the physicaly higher segment is segment 2
    '''

    def __init__(self, print_queue, upper_len=1, lower_len=1, upper_mass=1, lower_mass=1,
                 upper_length=1, lower_length=1, Kddd=1, Kdd=1, Kd=1, Kp=1, dampening_lower=0,
                 dampening_upper=0):
        '''
        set all constants for the arm
        '''
        # load everything
        self.l2 = upper_length
        self.l1 = lower_length
        self.m2 = upper_mass
        self.m1 = lower_mass
        self.i2 = upper_len
        self.i1 = lower_len
        self.Kddd = Kddd
        self.Kdd = Kdd
        self.Kd = Kd
        self.Kp = Kp

        self.dampening = [0, 0]
        self.velocity = [0, 0]

        self.print_queue = print_queue

        self.dampening[0] = dampening_lower
        self.dampening[1] = dampening_upper

        self.q1 = 0
        self.q2 = 0
        self.t = 0

        self.g = 9.81

        self.Ldot = 0
        self.Ldotdot = 0
        self.last_time = 0
        self.lastq1 = self.q1
        self.lastq2 = self.q2
        self.lastslope_q1 = 0
        self.lastslope_q2 = 0
        self.last_x = 0
        self.x = 0
        self.lastx_slope = 0
        self.x_slope = 0
        self.last_x_acceleration = 0
        self.x_thrid_derivative = 0

    def calculate(self, current_time=-9999):

        if self.ideal_q2 != -999:
            self.tD = self.get_tD(self.m2, self.i2, self.q1, self.ideal_q2)
        else:
            self.tD = 1 * self.m2 * self.i2 * self.g * math.cos(self.q1 + self.q2)
        #         self.tD = 1 * self.m2 * self.i2 * self.g * math.cos(math.pi/2 + self.q2)

        if current_time == -9999:
            self.current_time = time.clock()
        else:
            self.current_time = current_time

        self.calculate_acceleration(self.current_time)
        try:
            self.print_queue.put("x slope        = " + str('{0:.16f}'.format(self.x_slope)))
            self.print_queue.put("x acceleration = " + str('{0:.16f}'.format(self.x_acceleration)))
            self.print_queue.put("x third derivative = " + str('{0:.16f}'.format(self.x_thrid_derivative)))

            self.print_queue.put("Kdd = " + str('{0:.16f}'.format(self.Kdd * self.x_acceleration)))
            self.print_queue.put("Kd  = " + str('{0:.16f}'.format(self.Kd * self.x_slope)))
            self.print_queue.put("Kp  = " + str('{0:.16f}'.format(self.Kp * self.get_center_of_gravity())))

            self.t = 1 * (
                (self.Kddd * self.x_thrid_derivative + self.Kdd * self.x_acceleration + self.Kd * self.x_slope) +
            self.Kp * self.get_center_of_gravity() + 1.0 * self.tD)
            self.t = self.t - (self.dampening[1] * self.velocity[1])
            self.print_queue.put("de-dampening added " + str((self.dampening[1] * self.velocity[1])))
            self.print_queue.put("X = " + str('{0:.10f}'.format(self.get_center_of_gravity())))
        except:
            pass

        # holding torque

    def get_tD(self, m2, i2, q1, q2):
        return m2 * i2 * self.g * math.cos(q1 + q2)

    # self explanitory
    def get_center_of_gravity(self):
        a = math.cos(self.q1)
        b = self.l1 * self.m1
        x = a * b
        y = ((math.cos(self.q1 + self.q2) * self.l2) + math.cos(self.q1) * self.l1) * self.m2

        return (x + y)

    # for calculating L (angular momentum)
    def get_angular_position(self):
        q22 = math.pi - self.q2
        a = math.sqrt((self.l1 * self.l1 + self.l2 * self.l2) - (2.0 * self.l1 * self.l2 * math.cos(self.q2)))
        q11 = math.asin((math.sin(self.q2) * self.l1) / a)
        q12 = self.q1 / q11
        c = a * math.sin(q12)
        b = math.sqrt(c * c + a * a)
        self.center_of_mass_angle = math.atan(c / b)

    def calculate_acceleration(self, time):
        #         figure out accelerations
        self.print_queue.put("tD = " + str(self.tD))

        self.x = self.get_center_of_gravity()
        self.print_queue.put("x = " + str(self.x))

        self.x_slope = self.slope(self.last_time, self.current_time, self.last_x, self.x)

        self.x_acceleration = self.slope(self.last_time, self.current_time, self.lastx_slope, self.x_slope)

        self.x_thrid_derivative = self.slope(self.last_time, self.current_time, self.last_x_acceleration,
                                             self.x_acceleration)

        self.last_x_acceleration = self.x_acceleration

        self.lastx_slope = self.x_slope

        self.last_x = self.x

    #         self.lower_slope = self.slope(self.last_time, self.current_time, self.lastq1, self.q1)
    #         self.upper_slope = self.slope(self.last_time, self.current_time, self.lastq1, self.q1)
    #
    #         self.lower_acceleration = self.slope(self.last_time, self.current_time, self.lastslope_q1, self.lower_slope)
    #
    #         self.lastslope_q1 = self.lower_slope
    #         self.lastslope_q2 = self.upper_slope
    #
    #         self.lastq1 = self.q1
    #         self.lastq2 = self.q2

    # used to input
    def set_readings(self, lower_angle, upper_angle, future_q2=-999):
        self.q1 = lower_angle
        self.q2 = upper_angle
        #         if future_q2 != -999:
        #             self.ideal_q2 = future_q2
        #         else:
        #             self.ideal_q2 = -999
        self.ideal_q2 = future_q2

    def slope(self, last_time, current_time, last_measurement, current_measurement):
        try:
            return (current_measurement - last_measurement) / (current_time - last_time)
        except:
            pass

    def load(self):
        pass

    def get_dampening_compensation(self, joint):
        return self.velocity[joint] * self.dampening[joint]

    def get_holding_torque(self):
        return self.tD

    def get_balance_torque(self):
        return self.t

    def get_horizontal_displacement(self):
        return self.x

    def get_horizontal_displacement_slope(self):
        return self.x_slope

    def set_dampening(self, joint, magnitude):
        self.dampening[joint] = magnitude

    def set_velocity(self, joint, velocity):
        self.velocity[joint] = velocity

# archive (none of this code is used, and most of it is somehow broken)


#     def get_L(self):    
#         return ((math.cos(self.q1/self.l1)*self.m1) + math.cos(self.q1 + 1 * self.q2) 
#         * (self.l2 + math.cos(self.q1)) * self.m2)
# 
# #         L3 = math.pow((math.pow(self.l1,2) + math.pow(self.l2,2) - 2*self.l1*self.l2*math.cos(180-self.q2)),.5)
# #         print "L3 = " + str(L3)
# #         Q4 = math.acos((math.pow(self.l2,2) - math.pow(self.l1,2) - math.pow(L3, 2)) / (-2 * self.l1 * L3))
# #         Q5 = self.q1 - Q4
# #         print "q5 = " + str(Q4)
# #         return Q5
