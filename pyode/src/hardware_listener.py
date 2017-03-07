__author__ = 'David Smart'

import numpy as np
import time
import hardware


def print_tupple(tup, new_line=False, precision=5):
    print_tupple_helper(tup, precision)
    if new_line:
        print()


def print_tupple_helper(tup, precision):
    if type(tup).__name__ == 'tuple' or type(
            tup).__name__ == 'list' or type(tup).__name__ == 'ndarray':
        for i in tup:
            if type(i).__name__ == 'tupple' or type(
                    i).__name__ == 'list' or type(i).__name__ == 'ndarray':
                print(('['), end=' ')
                print_tupple_helper(i, precision),
                print((']'), end=' ')
            else:
                if i == 0.0:
                    i = 0
                if i >= 0.0:
                    print("", end=' ')  # add single space for formatting
                print((('%.' + str(precision) + 'f') % i + ","), end=' ')
    else:
        print((str(tup) + str(precision) + 'f'), end=' ')


robot = hardware.acrobot()
n = 0
while True:
    print(n, ",", end=' ')
    print_tupple(robot.write_read(0)[0:4], new_line=True, precision=5)
    time.sleep(0.001)
    n += 1
