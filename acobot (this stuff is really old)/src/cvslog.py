import numpy as np
from scipy import polyfit
from serial import Serial
import time                             # sleep
import signal                           # signal handling
import os
import sys
from struct import unpack
from pylab import *
import re
import logger
import hold_angle

import logging
import balance1

def count2time(t):
   # """ Returns seconds from micro controller time . """
    return t/16e6;

def a2r(angle):
    #""" Converts an angle in the abs(range) of 0 to max to radians. """
    return angle/512.0 * np.pi

def deg_to_rad(angle):
    return angle * (np.pi / 180)

def rad_to_deg(angle):
    return angle * (180 / np.pi)

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


        """TODO: add in windows com port auto-selection"""
        # Choose first available FTDI device
#         if dev:
#             self.dev = dev
#         else:
#             try:
#                 self.dev = filter(lambda a: 'ttyUSB' in a, os.listdir('/dev'))[0]
#             except:
#                 print "No FTDI devices found"
#                 sys.exit(1)
        dev = "COM2"

        #"""Start serial port"""
        # Try connecting to it
        try:
#             self.s = Serial(dev, 1000000, timeout=1)
            self.s = Serial(dev, 1000000, timeout=1)
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
        time =  (unpack(">H", self.s.read(2)))
#         time = 0
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


def constrain(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

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
        after = milli_time()
        print "time elapsed =",
        print after-before
        #     print "microcontroler took to respond: ",
        #     print int(vals[4].encode('hex'))
        print "Lower velocity: %f,  Lower angle: %f"%(vals[0], vals[2])
        print "Upper velocity: %f,  Upper angle: %f"%(vals[1], vals[3])

        print("time elapsed on microcontroller: %f")%vals[4]

        input = raw_input("power (not 127) ")
        if str(input) == "exit":
                a.__del__()
                exit = True


        if str(input) == "log":
            seconds = int(raw_input("how many seconds to log for?"))
            log = logger.log(directory = "logs")

            starting_time = time.clock()
            log.write("lower angle")
            log.write("upper angle")
            log.write("lower velocity")
            log.write("upper velocity")
            log.write("mills since start")
            log.write_line()
            starting_mills = milli_time()
            while starting_time + seconds > time.clock():
                vals = a.write_read(0)
                log.write(vals[2])
                log.write(vals[3])
                log.write(vals[0])
                log.write(vals[1])
                log.write(milli_time() - starting_mills)
                log.write_line()
            log.close()


        if(str(input) == "bench"):

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

            time.sleep(0)

        if str(input).split(" ")[0]  == "hold":
            try:
                print""
                
                args = str(input).split(" ")
                angle_deg = int(args[1])
            except:
                angle_deg = 0
            
#             angle_deg = int(raw_input("what angle to hold (degrees)"))
#             angle_deg = constrain(angle, -30, 30)
            angle_deg = max(angle_deg, -30)
            angle_deg = min(angle_deg, 30)
            angle_rad = deg_to_rad(angle_deg)
            print "holding " + str(angle_rad) + " radians"
            
            holder = hold_angle.hold(angle = angle_rad, acrobot = a)
            
            seconds_to_hold = int(args[2])
            start_time = time.clock()
            while time.clock() - start_time < seconds_to_hold:
                holder.itterate(angle_rad)
                time.sleep(0.0) 
            a.write_read(0)
            holder.close()

        if str(input).split(" ")[0] == "balance":
            balance = balance1.balance(a, seconds_to_hold = str(input).split(" ")[1])
            balance.go()

        try:
            input = int(input)
        except:
            input = 0
        print input,
        print "written"










