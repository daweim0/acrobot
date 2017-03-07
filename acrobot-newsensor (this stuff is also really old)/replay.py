#!/usr/bin/env python

from acrobot import *
import csv
import random
import sys


def replay_run(powers):
    acro = acrobot()
    i = 0
    power = 0
    Hz = 0
    X = acro.write_read(power) + [power, Hz] # write_read stop, get initial vals
    data = [X]
    for power in powers:
        start_time = time.time()
        
        # calculations
        Hz = 1./(time.time() - start_time)
        X = acro.write_read(power)
        total_time = time.time() - start_time
        slump = 1/20. - total_time
        if slump > 1/20.:
            slump = 1/20.
        time.sleep(1/20. - slump)
        
    acro.stop()
    
if __name__ == '__main__':
    # read in power settings
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "data/acrobot.csv"
    file = open(filename, "r")
    reader = csv.reader(file, delimiter='\t')
    powers = []
    for row in reader:
        powers += [int(float(row[-1]))]
        if(powers[len(powers)] == 0):
            powers[len(powers)] = 1;

    replay_run(powers)


