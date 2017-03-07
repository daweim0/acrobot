'''
Created on Jul 28, 2014

@author: David
'''

class derivatives(object):
    '''
    integrates using running totals
    '''


    def __init__(self, starting_value, starting_time, sample_pool = 1):
        self.last = starting_value
        self.current = starting_value
        self.last_time = starting_time
        self.current_time = starting_time
        self.sample_pool = sample_pool

    
    def slope(self, current, last, current_time, last_time):
        return  (current - last) / (current_time - last_time)
    
    def add(self, value, time):
        self.last = self.current
        self.current = int(value)
        self.last_time = self.current_time
        self.current_time = time
        self.last_slope = self.slope
        self.slope = self.slope(self.current, self.last, self.current_time, self.last_time)
        self.acceleration = self.slope(self.slope, self.last_slope, self.current_time, self.last_time)
        
    def get_slope(self):
        return self.slope
    
    def get_acceleration(self):
        return self.acceleration