'''
Created on Jul 15, 2014

@author: David
'''

import logger
import time
import pid
import hold_angle

class balance(object):
    '''
    Balances without any fancy trig. derives the angle to set the motor to by 
    using the error on the shoulder joint
    '''


    def __init__(self, acrobot, shoulder_center = 0, Kp = 1, Ki = 0, Kd = 0, logging = False, seconds_to_hold = 3):
        '''
        Constructor
        '''
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.a = acrobot
        
        self.motor_value = 0
        self.seconds_to_hold = seconds_to_hold
        
        self.pid = pid.PID()
        self.pid.setPoint(0)
        
        self.motor = hold_angle.hold(angle = 0, seconds = -1, log = False, acrobot = self.a)
    
        if logging:
            self.log = logger.log(directory = "hold_logs")
            self.log.write("upper angle")
            self.log.write("motor command")
            self.log.write("mills since start")
            self.log.write("holding angle = " + str(shoulder_center))
        else: self.log = None

    
    
    def go(self):
        self.last_values = self.motor.get_angles()
        
        start_time = time.clock()
        while time.clock() - start_time < self.seconds_to_hold:
#                 get angles
            self.last_values = self.motor.get_angles()
#                 do math
            
            self.motor_value = self.pid.update(self.last_values[2])
            
#                 write motor values
            self.last_values = self.motor.itterate(self.motor_value)
            time.sleep(0.0) 
                
                
                
                
            
            
            
            
            
            
            
            
            
            
            
            