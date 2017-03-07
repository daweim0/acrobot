#!/usr/bin/env python

import numpy as np
from serial import Serial
import curses                           
import time                             # sleep
import signal                           # signal handling
import os, sys
from struct import *                    # pack and unpack

try:
    dev = os.popen("ls /dev/ttyUSB*").next().strip()
except:
    dev = "/dev/ttyS0"
    
s = Serial(dev, 115200, timeout=1)

def quit_handler(signal, frame):
    # close serial connection
    s.write(pack("B",0))
    s.close()

    # end curses
    #curses.nocbreak()
    #stdscr.keypad(0)
    #curses.echo()
    #curses.endwin()
    
    print "Quitting."
    sys.exit(0)
    
signal.signal(signal.SIGINT, quit_handler)
signal.signal(signal.SIGTERM, quit_handler)

# Start curses
stdscr = curses.initscr()
#curses.noecho()                         # don't echo input
#curses.cbreak()                         # react immediately to input
stdscr.keypad(1)                        # get special values for special keys


def get_angles():
    s.flushInput()
    s.flushOutput()
    s.write(pack("B",0))

    # read 4 16-bit values in network byte order into an array
    # 0 - upper_high
    # 1 - upper_total
    # 2 - lower_high
    # 3 - lower_total
    vals = unpack("!4h", s.read(8))

    upper = 1026.0 * vals[0] / vals[1] - 1
    if upper > 1022: upper = 1023.0
    upper -= 512

    lower = 1026.0 * vals[2] / vals[3] - 1
    if lower > 1022: lower = 1023.0
    lower -= 512
    return [upper, lower]

upper, lower = get_angles()

total_upper = upper
total_lower = lower

while True:
    stdscr.clear()

    old_upper = upper
    old_lower = lower
    upper, lower = get_angles()
    delta_upper = upper - old_upper
    
    delta_lower = lower - old_lower

    # print graph lines
    for i in [0,1]:
        for c in [0,80]:
            stdscr.addstr(i,c,'|')
        for c in range(1,80):
            stdscr.addstr(i,c,'-')

    try:
        # print graph position
        stdscr.addstr(0,int((upper+512)*80/1023), "#")
        stdscr.addstr(1,int((lower+512)*80/1023), "#")
        
    except ValueError:
        stdscr.addstr(0,0,'Value Error: %f %f'%(upper, lower))
        quit_handler()

    power = 5 * int(abs(upper) - 5*abs(delta_upper))

    # safety first
    if power > 255: power = 255
    if power < 0: power = 0
    
    # # too far left
    # if upper < -2:
    #     s.write('r' + pack('B', power))
    # # too far right
    # elif upper > 2:
    #     s.write('l' + pack('B', power))

    # else:
    #     s.write('s')
        
    # print numbers
    stdscr.addstr(3,0, "Upper: %4.2f Lower: %4.2f Power: %3d"%(upper, lower, power))
    stdscr.addstr(4,0, "Delta: %3.2f"%(delta_upper))

    # tidy up
    stdscr.refresh()
    time.sleep(0.03)
    #s.write('s')
    



