import cPickle as pickle
from serial import Serial
from struct import *
from numpy import *
from acrobot import get_angle, a2r, count2time
import time

scon = Serial("COM9", 230400, timeout=1)
file = open("st_u_l.pkl",'wb')
LEN = 64

def get_reading(n = LEN):
    return unpack("!%dh"%n, scon.read(2*n))

def get_data(joint = 'upper', samples = 1):
    start = time.time()

    j_tmp = [()] * samples
    jp_tmp = [()] * samples

    start_read = time.time()

    # ask for data
    scon.flushInput()
    scon.flushOutput()
    scon.write(joint[0])
    # get readings
    for i in range(samples):
        j_tmp[i] = get_reading()
        jp_tmp[i] = get_reading()

    scon.write(']')

    end_read = time.time()

    # allocate space
    j = zeros(samples * LEN)
    jp = zeros(samples * LEN)

    # copy values over
    for i in range(samples):
        pos = LEN*i
        j[pos:pos+LEN] = j_tmp[i]
        jp[pos:pos+LEN] = jp_tmp[i]
    
    print "Total %f, xmit: %f"%(time.time() - start, end_read - start_read)
    return j, jp

# Capture some of data
samples = int(raw_input("Capture seconds? ")) * 16
print("Capturing upper joint")
up,upp = get_data('u',samples)
print("Capturing lower joint")
lo,lop = get_data('l',samples)

# Save it to st_u_l.pkl
pickle.dump(up,file)
pickle.dump(upp,file)
pickle.dump(lo,file)
pickle.dump(lop,file)
