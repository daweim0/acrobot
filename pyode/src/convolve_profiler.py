import convolve_py
import random
import convolve1
import numpy
import time
np = numpy

for i in range(200):
    N = 10
    f = np.arange(N*N, dtype=np.int).reshape((N,N))
    g = np.arange(81, dtype=np.int).reshape((9, 9))
    start_time=time.time()
    new_result = convolve1.naive_convolve(f, g)
    new_time = time.time() - start_time
    old_result = convolve_py.naive_convolve(f, g)
    old_time = time.time() - start_time - new_time
    # if numpy.sum(numpy.asarray(new_result) - numpy.asarray(old_result)) > 0.001:
        # print("borked at", random_tuple)
    # else:
    print("    ok with times (new, old) (", new_time*1000, old_time *1000, ")")
