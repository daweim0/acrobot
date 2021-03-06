import numpy as np
from scipy import polyfit
from serial import Serial
import time                             # sleep
import signal                           # signal handling
import os, sys
from struct import unpack
from pylab import *
import re

def count2time(t):
    """ Returns the fraction of a second represented by a certain count of
    the 16 bit timer. """
    return t/16e6;

def a2r(angle):
    """ Converts an angle in the abs(range) of 0 to max to radians. """
    return angle/512.0 * np.pi

def get_angle(high, total):
    angle = int(1026.0 * high / total - 1)
    if angle > 1022: angle = 1023
    return a2r(angle - 512)

def milli_time():
    return int(round(time.time() * 1000))

class acrobot:
    def __init__(self, dev=0):
        self.dev = ""
        self.power = 0
        self.lo_period = 16357          # approximates, but are temperature
        self.up_period = 16557          # dependent, so call .calibrate()
        self.LEN = 8                    # number of samples sent
        self.CLK = 16e6                 # ATMEGA128 clock freq
        self.MAX_SUM_COUNT = 4.0        # number of samples sent in a sum
        self.SAMPLES = 16.0             # nmber of samples between sums

        # Try connecting to it
        try:
            self.s = Serial("COM9", 230400, timeout=1)
            self.s.flushInput()
            self.s.flushOutput()
        except:
            print "Couldn't open connection to" + str(self.dev)
            sys.exit(1)

        # Calibrate for current temperature
        #self.calibrate()
            
        return

    def calibrate(self):
        self.lo_period = []
        self.up_period = []
        for i in range(20):
            self.s.write(chr(127))
            self.lo_period += self.get_reading()
            self.up_period += self.get_reading()

        self.lo_period = np.average(self.lo_period)
        self.up_period = np.average(self.up_period)
        return self.lo_period, self.up_period

    def __del__(self):
        pass

    def get_reading(self, n = 0):
        if not n:
            n = self.LEN
        return list(unpack("!%dH"%n, self.s.read(2*n)))

    # Find a best fit line for the slope, etc.
    # data input is in -512 to +512
    def find_slope(self, data, period):
        t = np.arange(self.LEN) * period * self.SAMPLES / self.CLK

        # Find a cubic fit to get a slope for the second to last data point.
        a,b,c,i = polyfit(t, data, 3)
        lt = t[-2]
        v = 3*a*lt**2 + 2*b*lt + c

        # Use mean of last three points for the position - this introduces a
        # small delay into the position data.
        av = np.average(data[-3:])
        return v, av

    def send(self, power):
        if power < 0: power = -power + (1 << 7) # encode power
        if power == 127: power = 126            # inhibit calibration
        self.power = power
        self.s.write(chr(power))

        # self.lo and self.up will contain the last LEN data points, oldest in
        # index 0. Each raw data point is the sum of 4 consecutive samples
        # (supersampling) and so we divide by 4 to get a closer approximation.
        self.lo = np.array(self.get_reading()) / self.MAX_SUM_COUNT
        self.up = np.array(self.get_reading()) / self.MAX_SUM_COUNT

        # Convert to angles
        for i in range(len(self.lo)):
            self.lo[i] = get_angle(self.lo[i], self.lo_period)
            self.up[i] = get_angle(self.up[i], self.up_period)

        v_lo, lo = self.find_slope(self.lo, self.lo_period)
        v_up, up = self.find_slope(self.up, self.up_period)
        return [v_lo, v_up, lo, up]

    # process data from arbitrary source
    def inject(self, los, ups):
        self.lo = los
        self.up = ups
        v_lo, lo = self.find_fit(los, self.lo_period)
        v_up, up = self.find_fit(ups, self.up_period)
        return [v_lo, v_up, lo, up]

    def get_vals(self):
        return self.send(self.power)

    def get_filter_vals(self):
        return self.filter_send(self.power)

    def stop(self):
        return self.send(0)

    def __str__(self):
        vals = self.get_vals()
        return "U: %3.4f [d %3.4f], L: %3.4f [d %3.4f], P: %d"%(vals[3], vals[1], vals[2], vals[0], self.power)


if __name__ == '__main__':
    a = acrobot()
    print "Calibrating..."
    a.calibrate()
    print "Lower period: %d,  Upper period: %d"%(a.lo_period, a.up_period)
    print "Reading current values..."
    before = milli_time()
    vals = a.send(0)
    after = milli_time()
    print "Lower velocity: %f,  Lower angle: %f"%(vals[0], vals[2])
    print "Upper velocity: %f,  Upper angle: %f"%(vals[1], vals[3])
    print "time elapsed: %f"%(after-before)
    

    exit = False
    input = 0
    while not exit:


        print "Reading current values..."
        before = milli_time()
        vals = a.send(input)
        after = milli_time()
        print "time elapsed =",
        print after-before
        #     print "microcontroler took to respond: ", 
        #     print int(vals[4].encode('hex'))
        print "Lower velocity: %f,  Lower angle: %f"%(vals[0], vals[2])
        print "Upper velocity: %f,  Upper angle: %f"%(vals[1], vals[3])
        
        input = int(raw_input("power (not 127) "))
        print input,
        print "written"
