'''
Created on Jul 16, 2014

@author: David
'''

class math(object):
    '''
    based on "Angular momentum Based Controller for Balancing an Inverted Double Pendulum" 
    by Mortzea Azad* and Roy Featherstone*
    
    All functions in this class are clean (no external operations)
    the physicaly higher segment is denoted with 2 while the physicaly lower segment 
    is denoted with 1
    '''


    def __init__(self, upper_len = 1, lower_len = 1, upper_mass = 1, lower_mass = 2,
                  upper_center_of_gravity = .5, lower_center_of_gravity = .7, kv = 1, kx = 1, kp = 1):
        '''
        set all constants for the arm
        '''
        #load everything
        self.l2 = upper_len
        self.l1 = lower_len
        self.m2 = upper_mass
        self.m1 = lower_mass
        self.i2 = upper_center_of_gravity
        self.i1 = lower_center_of_gravity
        self.kv = kv
        self.kx = kx
        self.kp = kp
        
        self.q1 = 0
        self.q2 = 0
        self.t = 0
        
        
    def calculate(self):
        l2 = self.l2
        l1 = self.l1
        m2 = self.m2
        m1 = self.m1
        i2 = self.i2
        i1 = self.i1
        q1 = self.q1
        q2 = self.q2
        kv = self.kv
        kp = self.kp
        kx = self.kx
        
        
        g = 9.81
        
        tD = m2 * i2 * g * math.cos(q1 + q2)
        
#         t = -kv * 
        
        
    
    def set_readings(self, upper_angle, lower_angle):
        pass
    
    def get_motor_power(self):
        pass