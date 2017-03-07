from serial import Serial
import time
import sys
import os
import numpy as np
from struct import unpack

import logger


def count2time(t):
    # """ Returns seconds from micro controller time . """
    return t / 16e6


def a2r(angle):
    # """ Converts an angle in the abs(range) of 0 to max to radians. """
    return angle / 512.0 * np.pi


def deg_to_rad(angle):
    return angle * (np.pi / 180)


def rad_to_deg(angle):
    return angle * (180 / np.pi)


class acrobot:
    # """initialization"""
    def __init__(self, dev=0):
        self.dev = ""
        self.power = 0
        self.lo_period = 16357  # approximates, but are temperature
        self.up_period = 16557  # dependent, so call .calibrate()
        self.LEN = 8  # number of samples sent
        self.CLK = 16e6  # ATMEGA128 clock freq
        self.MAX_SUM_COUNT = 4.0  # number of samples sent in a sum
        self.SAMPLES = 16.0  # number of samples between sums
        self.bytes_read = 0  # debug counter

        """TODO: add in windows com port auto-selection"""
        # Choose first available FTDI device
        if dev:
            self.dev = dev
        elif os.name == "posix":
            try:
                self.dev = "/dev/" + filter(lambda a: 'ttyUSB' in a, os.listdir('/dev'))[0]
            except:
                print "No FTDI devices found"
                sys.exit(1)
        else:  # probably running on windows
            self.dev = "COM3"
        # """Start serial port"""
        # Try connecting to it
        try:
            self.s = Serial(self.dev, 500000, timeout=1)
            self.s.flushInput()
            self.s.flushOutput()
        except:
            print "Couldn't open connection to " + str(self.dev)
            sys.exit(1)

        # Calibrate for current temperature
        self.calibrate()
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

    # """program ending cleanup"""
    def __del__(self):
        # self.write_read(0)                    # power off
        self.s.close()
        pass

    # """ returns theta 2 from sensor readings, returns radians"""
    # TODO: check for sensor rotation (Is it currently in use?)
    def get_angle(self, high, total):
        angle = int(1026.0 * high / total - 1)
        if angle > 1022: angle = 1023
        return a2r(angle - 512)

    # """By default reads 256 bytes"""
    # """TODO: check number encoding"""
    def get_reading(self, n=None):
        if n is None:
            n = self.LEN
        data = list(unpack(">" + "H"*n, self.s.read(2*n)))
        # print "bytes", self.bytes_read, "to", self.bytes_read + n*2, " = ", data
        self.bytes_read += n*2
        return data

    def find_slope(self, data, period):
        t = np.arange(self.LEN) * period * self.SAMPLES / self.CLK

        # Find a cubic fit to get a slope for the second to last data point.
        a, b, c, i = np.polyfit(t, data, 3)
        lt = t[-2]
        v = 3 * a * lt ** 2 + 2 * b * lt + c
        # Use mean of last three points for the position - this introduces a
        # small delay into the position data.
        av = np.average(data[-3:])
        return v, av

    # Request data then read it. originally named send
    def write_read(self, power):
        if power < 0: power = -power + (1 << 7)  # encode power "map(power, 0, 128, 128, 0)"
        if power == 127: power = 126  # inhibit calibration
        if power == 1: power = 2  # inhibit advanced commands
        self.power = power
        self.s.write(chr(power))

        # self.lo and self.up will contain the last LEN data points, oldest in
        # index 0. Each raw data point is the sum of 4 consecutive samples
        # (supersampling) and so we divide by 4 to get a closer approximation.
        self.lo = np.array(self.get_reading()) / self.MAX_SUM_COUNT
        self.up = np.array(self.get_reading()) / self.MAX_SUM_COUNT

        # Convert to angles
        for i in range(len(self.lo)):
            self.lo[i] = self.get_angle(self.lo[i], self.lo_period)
            self.up[i] = self.get_angle(self.up[i], self.up_period)

        v_lo, lo = self.find_slope(self.lo, self.lo_period)
        v_up, up = self.find_slope(self.up, self.up_period)
        return [v_lo, v_up, lo, up]

    # process data from sources other than the hardware
    def inject(self, los, ups):
        self.lo = los
        self.up = ups
        v_lo, lo = self.find_fit(los, self.lo_period)
        v_up, up = self.find_fit(ups, self.up_period)
        return [v_lo, v_up, lo, up]

    def get_vals(self):
        return self.write_read(self.power)

    # """TODO: Check for filter_send()'s existence"""
    def get_filter_vals(self):
        return self.filter_send(self.power)

    def stop(self):
        return self.write_read(0)

    def __str__(self):
        vals = self.get_vals()
        return "U: %3.4f [d %3.4f], L: %3.4f [d %3.4f], P: %d" % (vals[3], vals[1], vals[2], vals[0], self.power)

    def set_led(self, input):
        self.s.write(chr(0x01))  # general command prefix
        self.s.write(chr(11))  # set led prefix
        if input == "red":
            self.s.write(chr(0x04))
        elif input == "green":
            self.s.write(chr(0x02))
        elif input == "blue":
            a.s.write(chr(0x01))
        elif ord(chr(input)) == input:  # check to see if input is a byte
            self.s.write(chr(input))
        else:
            raise Exception("set_led passed illegal argument", input)

    def ping(self):
        self.s.write(chr(0x01))
        self.s.write(chr(12))
        return self.s.read(1)


def milli_time():
    return time.clock() * 1000


def constrain(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n


def average(tupl):
    sum = 0.0
    for i in tupl:
        sum += i
    return sum / len(tupl)


# prints nested tupples with better formatting
def print_tupple(tup, new_line=False, precision=3):
    print_tupple_helper(tup, precision)
    if new_line:
        print


def print_tupple_helper(tup, precision):
    if type(tup).__name__ == 'tuple' or type(
            tup).__name__ == 'list' or type(tup).__name__ == 'ndarray':
        for i in tup:
            if type(i).__name__ == 'tupple' or type(
                    i).__name__ == 'list' or type(i).__name__ == 'ndarray':
                print('['),
                print_tupple_helper(i, precision),
                print(']'),
            else:
                print(('%.' + str(precision) + 'f') % i + ","),
    else:
        print(str(tup) + str(precision) + 'f'),


def set_led(input):
    a.s.write(chr(0x01))  # general command prefix
    a.s.write(chr(11))  # set led prefix
    if input == "red":
        a.s.write(chr(0x04))
    elif input == "green":
        a.s.write(chr(0x02))
    elif input == "blue":
        a.s.write(chr(0x01))
    elif ord(chr(input)) == input:  # check to see if input is a byte
        a.s.write(chr(input))


def ping():
    a.s.write(chr(0x01))
    a.s.write(chr(12))
    return a.s.read(1)

if __name__ == '__main__':
    print "alive"
    a = acrobot()
    a.stop()

    set_led("blue")
    time.sleep(.5)
    set_led("green")

    ping_reply = ord(ping())
    if ping_reply != 18:
        print "pint returned " + str(ping_reply) + " instead of 18, aborting"
        sys.exit(1)
    else:
        print "ping sucessfull"

    a.stop()

    exit = True
    input = 0
    while not exit:

        print "Reading current values..."
        before = milli_time()
        vals = a.write_read(input)
        after = milli_time()
        print "time elapsed =",
        print after - before
        #     print "microcontroler took to respond: ",
        #     print int(vals[4].encode('hex'))
        print "Lower velocity: %f,  Lower angle: %f" % (vals[0], vals[2])
        print "Upper velocity: %f,  Upper angle: %f" % (vals[1], vals[3])


        input = raw_input("power (not 127) ")
        if str(input) == "exit":
            a.__del__()
            exit = True

        if str(input) == "log":
            seconds = int(raw_input("how many seconds to log for?"))
            log = logger.log(directory="logs")

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

        if (str(input) == "bench"):

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
            print(totalTimeSum / repititions)
            print("serial converter time"),
            print(serialTimeSum / repititions)
            print("micro time = "),
            print(microTimeSum / repititions)
            print

            time.sleep(0)

        try:
            input = int(input)
        except:
            input = 0
        print input,
        print "written"
