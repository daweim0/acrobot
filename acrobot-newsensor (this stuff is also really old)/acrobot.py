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
   # """ Returns seconds from micro controller time . """
    return t/16e6;

def a2r(angle):
    #""" Converts an angle in the abs(range) of 0 to max to radians. """
    return angle/512.0 * np.pi

#""" returns theta 2 from sensor readings, returns radians"""
#TODO: check for sensor rotation (Is it currently in use?)
def get_angle(high, total):
    angle = int(1026.0 * high / total - 1)
    if angle > 1022: angle = 1023
    return a2r(angle - 512)

class acrobot:
   # """initialization"""
    def __init__(self, dev=0):
        self.dev = ""
        self.power = 0
        self.lo_period = 16357          # approximates, but are temperature
        self.up_period = 16557          # dependent, so call .calibrate()
        self.LEN = 8                    # number of samples sent
        self.CLK = 16e6                 # ATMEGA128 clock freq
        self.MAX_SUM_COUNT = 4.0        # number of samples sent in a sum
        self.SAMPLES = 16.0             # nmber of samples between sums


        #"""TODO: add in windows com port auto-selection"""
        # Choose first available FTDI device
#         if dev:
#             self.dev = dev
#         else:
#             try:
#                 self.dev = filter(lambda a: 'ttyUSB' in a, os.listdir('/dev'))[0]
#             except:
#                 print "No FTDI devices found"
#                 sys.exit(1)
        dev = "COM16"

        #"""Start serial port"""
        # Try connecting to it
        try:
#             self.s = Serial(dev, 230400, timeout=1)
            self.s = Serial(dev, 250000, timeout=1)
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
    
    #"""program ending cleanup"""
    def __del__(self):
#         self.write_read(0)                    # power off
        self.stop()
        self.s.close()

    
    #"""By default reads 256 bytes"""
    #"""TODO: check number encoding"""
    def get_reading(self, n = 0):
        if not n:
            n = self.LEN
        return list(unpack("!%dH"%n, self.s.read(2*n)))

    #  Find a best fit line for the slope, etc.
    #data input is in -512 to +512
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

    #Request data then read it. originally named send
    def write_read(self, power):
        if power < 0: power = -power + (1 << 7) # encode power "map(power, 0, 128, 128, 0)"
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
        self.s.read(1)
#        time =  (unpack(">H", self.s.read(2)))
        time = [0,0,0,0,0,0,0]
        return [v_lo, v_up, lo, up, int(time[0])/16000.0]

    # process data from arbitrary source
    def inject(self, los, ups):
        self.lo = los
        self.up = ups
        v_lo, lo = self.find_fit(los, self.lo_period)
        v_up, up = self.find_fit(ups, self.up_period)
        return [v_lo, v_up, lo, up]

    def get_vals(self):
        return self.write_read(self.power)

    #"""TODO: Check for filter_send()'s existence"""
    def get_filter_vals(self):
        return self.filter_send(self.power)

    def stop(self):
        return self.write_read(0)

    def __str__(self):
        vals = self.get_vals()
        return "U: %3.4f [d %3.4f], L: %3.4f [d %3.4f], P: %d"%(vals[3], vals[1], vals[2], vals[0], self.power)

def milli_time():
    return time.clock() * 1000


if __name__ == '__main__':
    print "alive"
    a = acrobot()
    print "Calibrating..."
    a.calibrate()
    print "Lower period: %d,  Upper period: %d"%(a.lo_period, a.up_period)

    exit = False
    input = 0
    while not exit:


        print "Reading current values..."
        before = milli_time()
        vals = a.write_read(input)
        a.s.flushInput()
        after = milli_time()
        print "time elapsed =",
        print after-before
        #     print "microcontroler took to respond: ", 
        #     print int(vals[4].encode('hex'))
        print "Lower velocity: %f,  Lower angle: %f"%(vals[0], vals[2])
        print "Upper velocity: %f,  Upper angle: %f"%(vals[1], vals[3])
        
        print("time elapsed on microcontroller: %f")%vals[4]
        
        input = raw_input("power (not 127) ")
        print input,
        print "written"

        if str(input) == "exit":
            a.stop()
            a.s.close()
            exit = True
        
        elif(input > 127):
            
            input = 0
            
            repititions = 500
            
            microTimeSum = 0
            serialTimeSum = 0
            totalTimeSum = 0
            
            timeTakenBefore = milli_time()
            
            for x in range(0, repititions):
                before = milli_time()
                vals = a.write_read(0)
                after = milli_time()
                                
                totalTimeSum += after - before
                microTimeSum += vals[4]
                serialTimeSum += after - before - vals[4]
            timeTakenAfter = milli_time()
                
            print
            print("total time = "),
            print( totalTimeSum / repititions)
            print("serial converter time"),
            print(serialTimeSum / repititions)
            print("micro time = "),
            print(microTimeSum / repititions)
            print
            
            time.sleep(0.001)
        if not exit:
            input = int(input)

                
        
        
        
        
        
        
        

    
