'''
Created on Jul 23, 2014

@author: David
'''

class dampener(object):
    '''
    aids in the dampening of joints
    '''


    def __init__(self, dampening_coeficient, joint):
        self.joint = joint
        self.K = dampening_coeficient
        self.velocity = 0
    
    def tick(self):
#         print "angle velocity = " + str(self.velocity)
        self.joint.addTorque(-1 * self.K * self.velocity)
#         print "dampener added " + str(-1 * self.K * self.velocity)
    
    def setK(self,k):
        self.K = k
    
    def getK(self):
        return self.K
    
    def setVelocity(self, velocity):
        self.velocity = velocity
    
    def __slope(self, last_time, current_time, last_measurement, current_measurement):
        return  (current_measurement - last_measurement) / (current_time - last_time)    