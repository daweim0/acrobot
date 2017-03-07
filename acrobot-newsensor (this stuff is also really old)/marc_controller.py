#!/usr/bin/env python
if __name__ == '__main__':
    print "Loading modules."
    
import numpy as np                      # array functions
from serial import Serial               # serial port connection
import time                             # sleep()
import signal                           # signal handling
from struct import *                    # pack() and unpack()
from math import *                      # trig functions
import csv
import sys, os
from acrobot import *
from pylab import *
import re

#################################################
def read_data():
    loghypers_file = open('loghypers.txt', 'r');
    loghypers = np.array([float(j.strip()) for j in loghypers_file.read().split()])

    beta_file = open('beta.txt', 'r');
    beta = np.array([float(j.strip()) for j in beta_file.read().split()])

    training_inputs_file = open('training_inputs.txt','r');
    training_reader = csv.reader(training_inputs_file, delimiter=' ', skipinitialspace=True)
    training_inputs = np.array([[float(item) for item in row]for row in training_reader])

    return [loghypers, beta, training_inputs]

def trigaug(x):
    return x[:2] + [sin(x[2]), cos(x[2]), sin(x[3]), cos(x[3])]


def get_covariance(loghypers, training_inputs, y):

    D = len(loghypers)-2

    alpha = np.exp(2*loghypers[D])
    ell = np.exp(loghypers[:D])

    vec = np.array([alpha*np.exp(-0.5*sum(((y-i)/ell)**2)) for i in training_inputs])

    return vec

def get_mean_fct(beta, loghypers, training_inputs, y):
    k = get_covariance(loghypers, training_inputs, y)
    m = sum(beta*k)
    return m

def get_control(beta, loghypers, training_inputs, x):
    m = get_mean_fct(beta, loghypers, training_inputs, trigaug(x))
    power = int(np.round(127*sin(m)))
    return power

##################################################
# Main Function
if __name__ == '__main__':
    print "Reading control data."
    # 0 get parameters (pre-load)
    loghypers, beta, training_inputs = read_data();
    power = 0
    stuff_time = 0                       # time to do stuff
    # v_lo, v_up, lo, up, power
    data = []
    acro = acrobot()
    print "Calibrating sensor periods."
    acro.calibrate()
    X = []
    
    # warmup the python interpreter
    print "Warming up the interpreter."
    for i in range(5):
        X = acro.write_read(0)
        get_control(beta,loghypers,training_inputs, X)
    
    i = 0;
    data += [X + [power]]

    # Run the balance routine
    print "Starting run."
    run_time = time.time()
    #for j in range(20):
    while True:
        # 1 Send signal and sleep for 0.05 seconds [start of period].
        send_time = time.time()
        acro.write_read(power)
        send_time = time.time() - send_time

        sleep_time = 1/20. - stuff_time - send_time
        if sleep_time < 0: sleep_time = 0
        time.sleep(sleep_time)
        
        # 2 read angles etc from pendulum --> X.  Power is sent just
        # to read the state. [end of period]
        stuff_time = time.time()
        X = acro.write_read(power)

        # Halt if dead
        if abs(X[2]) > np.pi/4 or abs(X[3]) > np.pi/2:
            break

        # 3 Compute the next signal and log current state with next signal
        power = get_control(beta, loghypers, training_inputs, X)
        data += [X + [power]]
        i += 1
        stuff_time = time.time() - stuff_time

    # Power off
    run_time = time.time() - run_time
    acro.stop()
    print "Run lasted for [%d] iterations, %1.2f seconds"%(i, run_time)

    # Save results to file
    # Get the major number for this run, 1 more than the number of old
    # controller input.
    major = int(list(os.popen("ls old_controller/beta*|wc -l"))[0].strip()) + 1

    # Find out what the minor number should be
    files = list(os.popen("ls data/acrobot%d*"%major))
    if files:                       # i.e. ls returned results
        minor = int(re.search("(\d+)_(\d+)", files[-1]).groups()[1]) + 1
    else:
        minor = 1
    filename = "data/acrobot%d_%d.csv"%(major, minor)
    try:
        file = open(filename, "w")
        writer = csv.writer(file, delimiter='\t')
    except:
        print "Can't open csv file"
        print filename
        sys.exit(1)

    print "Saving data to " + filename
    for line in data:
        writer.writerow(line)
    file.close()

    data = array(data)
    vlo = data[:,0]
    vup = data[:,1]
    lo = data[:,2]
    up = data[:,3]
    power = data[:,4]

    figure(1)
    subplot(211)
    title("Upper Joint")
    t = arange(len(data))
    plot(t, up, label='Position')
    plot(t, vup, 'r', label='Velocity')
    plot(t, power/100, label='Power/100')
    legend()

    subplot(212)
    title("Lower Joint")
    t = arange(len(data))
    plot(t, lo, label='Position')
    plot(t, vlo, 'r', label='Velocity')
    legend()
    show()
