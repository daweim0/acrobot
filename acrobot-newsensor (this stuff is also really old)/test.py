#!/usr/bin/env python

import csv
import time
from acrobot import *
from pylab import *
import cPickle as pickle

a = acrobot()
pkl_file = open('st_u_l.pkl','rb')
raw_up = pickle.load(pkl_file)
raw_upp = pickle.load(pkl_file)
raw_lo = pickle.load(pkl_file)
raw_lop = pickle.load(pkl_file)
a.up_period = average(raw_upp)
a.lo_period = average(raw_lop)
t = arange(len(raw_lo))
t = t * a.lo_period / a.CLK
Hz = 40.0

# convert absolute angle to radians
def conv(data, period):
    vals = zeros(len(data))
    for i in range(len(data)):
        vals[i] = get_angle(data[i], period)
    return vals

lo = conv(raw_lo, a.lo_period)
up = conv(raw_up, a.up_period)

# data and N should be powers of 2
def get_samples(data = lo, N = 32):
    conv = 1024/Hz
    rate = int(len(data)/conv)
    samples = []
    sample_t = []
    for i in range(rate):               # every rate samples
        for j in range(N):              # save N samples
            index = int(i*conv + j - 1 )
            if index < len(data):
                samples += [data[index]]
                sample_t += [t[index]]

    return [samples, sample_t]

def simulate(lo, up, t, N = 32):
    vecs = []
    for i in range(len(lo)/N):
        start = i*N
        end = i*N+N
        vecs += [array(a.inject(lo[start:end],up[start:end]) + [t[end-1]])]
    return array(vecs)

def integrate(slope, start, period):
    area = zeros(len(slope))
    area[0] = start
    for i in range(1,len(slope)):
        area[i] = area[i-1] + slope[i]*period
    return area

figure(10)
subplot(211)
plot(t,up/512*pi,color="black",label="raw")
subplot(212)
plot(t,up/512*pi,color="black",label="raw")

trials = arange(1,5)
sizes = 2**(trials+1)
for i in trials:
    N = sizes[i-1]
    a.LEN = N
    sub_lo, sub_t = get_samples(lo, N)
    sub_up, _ = get_samples(up, N)          # same sub_t as for sub_lo

    # calculate stuff
    vecs = simulate(sub_lo, sub_up, sub_t, N)

    rt = vecs[:,4]
    rlo = vecs[:,2]
    rlov = vecs[:,0]
    rup = vecs[:,3]
    rupv = vecs[:,1]
    rupc = integrate(rupv, rup[0], 1/Hz)

    # draw results
    #figure(1)
    #subplot(211)
    #title("Lower Joint")
    #plot(t,lo)
    #plot(sub_t, sub_lo, '.')
    #plot(rt, rlo, label='Position')
    #plot(rt, rlov, label='Velocity')
    #legend()

    figure(1)
    subplot(2,2,i)
    title("Upper Joint N=%d (%.1f Hz)"%(N,Hz) )
    plot(t,up*pi/512, label='measured')
    plot(rt,rupc, label='integrated')
    #plot(sub_t, sub_up, '.')
    plot(rt, rup, label='smoothed')
    #plot(rt, rupv, label='Velocity')
    legend()
    xlabel("Time (s)")
    ylabel("Radians")

    figure(10)
    subplot(211)
    title("Integrated Velocity : Absolute Difference (%.1f Hz)"%Hz)
    dif = rup - rupc
    plot(rt, dif, label='N = %d'%N)
    legend()
    xlabel("Time (s)")
    ylabel("Radians")

    subplot(212)
    title("Integrated Velocity: Cumulative Difference (%.1f Hz)"%Hz )
    cum_dif = zeros(len(dif))
    cum_dif[0] = dif[0]
    for i in range(1,len(dif)):
        cum_dif[i] = cum_dif[i-1] + dif[i]
    cum_dif = np.abs(cum_dif)
    plot(rt, cum_dif, label='N = %d'%N)
    legend()
    xlabel("Time (s)")
    ylabel("Radians")

show()
