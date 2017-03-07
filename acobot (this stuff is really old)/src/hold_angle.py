'''
Created on Jul 14, 2014

@author: David
'''

import pid
import logger
import time

def constrain(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

class hold:

    def __init__(self, angle = 0, seconds = 0, acrobot = None, p = 200 , i = 6, d = 200, log = False):
        self.set_angle = angle
        if(seconds > 0):
            self.seconds = seconds
        self.a = acrobot
        self.power = 0
        self.p_const = p
        self.i_const = i
        self.d_const = d     
        self.log_on = log   
#         set up pid controller
        self.pid = pid.PID(P=self.p_const, I=self.i_const, D=self.d_const, Integrator_max=120, Integrator_min=-120, Integrator_decrease_boost = 1)
        self.pid.setPoint(angle)
        
        if self.log_on:
            self.log = logger.log(directory = "hold_logs")
            
            self.log.write("upper angle")
            self.log.write("motor command")
            self.log.write("mills since start")
            self.log.write("set angle = " + str(angle))
        
        self.start_time = time.clock()
        
        
# returns current motor angles and velocities
    def itterate(self, angle = -999):
        if angle != -999:
            self.angle = angle
        self.angles = self.write(self.angle)
        
        if self.angles[3] ==  self.set_angle:
            print "no diff"
            
        self.power = self.pid.update(self.angles[3])
        self.angles = self.write(self.power)
        print "writing " + str(self.power) + " at angle " + str(self.angles[3])
        
        if self.log_on:
            self.log.write(self.angles[3])
            self.log.write(self.power)
            self.log.write(time.clock() - self.start_time)
            self.log.write_line()
    
        return self.angles
#         Old itterate function (proportional control)
        
#     def itterate(self, angle = -999):
#         if angle != -999:
#             self.angle = angle
#         angles = self.write(self.angle)
#         if angles[3] ==  self.set_angle:
#             self.write(0)
#             print "no diff"
#         else:
#             diff = self.angle - angles[3]
#             self.power = diff * self.p_const
#             self.write(self.power)
#             print "wrote " + str(self.power)
        
        
    def write(self, angle):
        returns = self.a.write_read(int(constrain(angle, -120, 120)))
        return returns
    
    def get_angles(self):
        return self.write(0)
    
    def close(self):
        if self.log_on:
            self.log.close()

