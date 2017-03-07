#!/usr/bin/env python

from acrobot import *
from pylab import *
import csv
import random

acro = acrobot()
period = 1./20

def data_run():
    power = 0
    X = acro.write_read(power) + [power] # write_read stop, get initial vals
    data = [X]
    while abs(X[2]) < np.pi/4 and abs(X[3]) < np.pi/2:
        start_time = time.time()
        print("alive")
        # Generate random data
        power = random.randint(-127,126)

        # calculations
        X = acro.write_read(0) + [power]
        data += [X]
        slump = time.time() - start_time
        if slump > period:
            slump = period
        time.sleep(period - slump)
        
    data = data[:-1]                    # remove point that stopped us

    acro.stop()
    return array(data)
    
if __name__ == '__main__':
    count = 1
    while not raw_input("(run %2d) Ready to go? "%(count)):
        try:
            file = open("data/random_balance_%02d.csv"%(count), "w")
            writer = csv.writer(file, delimiter='\t')
        except e:
            print "Can't open csv file"
            sys.exit(1)
        
        start_time = time.time()
        data = data_run()
        end_time = time.time()
        for line in data:
            writer.writerow(line)

        figure()
        grid(False)
        title("Run " + str(count))
        xlabel("Time (s)")
        ylabel("Angle (rad)")
        t = arange(len(data[:,0])) * period
        plot(t, data[:,0],'.-c',label="Lower Velocity",alpha=0.5)
        plot(t, data[:,1],'.-g',label="Upper Velocity",alpha=0.5)
        plot(t, data[:,2],'.-r',label="Lower Angle",lw='2')
        plot(t, data[:,3],'.-b',label="Upper Angle",lw='2')
        plot(t, data[:,4]/20,':',label="Power / 20")
        plot([t[0],t[-1]],[pi/4,pi/4],'r--',alpha=0.5)
        plot([t[0],t[-1]],[-pi/4,-pi/4],'r--',alpha=0.5)
        plot([t[0],t[-1]],[pi/2,pi/2],'b--',alpha=0.5)
        plot([t[0],t[-1]],[-pi/2,-pi/2],'b--',alpha=0.5)
        legend()
        show()
        
        count += 1
    
