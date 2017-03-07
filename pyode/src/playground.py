import time
import numpy as np
import hardware
import logger
import math
import random
random.seed = 9001

a = None


def main() -> None:
    """
    Main method of playground. Takes user input and sends it to the acrobot.
    """
    global a
    print("alive")
    a = hardware.acrobot()
    # a.stop()

    exit = False
    input_str = 0
    gyro_vals = [0, 0, 0, 0, 0, 0]

    # print('system_time', end=' ')
    # print("microcontroler_latency", end='')
    # print(" Upper_velocity Upper_angle", end='')
    # print(" Lower_velocity Lower_angle", end='')
    # print(" gyro_val", end='')
    # print(" torque", end='\n')

    print('torque a v x')

    last_torque = 1
    ticks = 0
    step_num = 0
    completed_motions = 0

    a.set_led("blue")
    # time.sleep(.5)
    a.set_led("green")

    initial_time = time.time()
    cycle_start_time = time.time()

    iterations = 100
    n_ticks = 37

    return_data = np.zeros([2, n_ticks+1, iterations])


    dt = 0.1

    input_str = 0

    before = time.clock()
    vals = a.write_read(input_str)
    after = time.clock()
    serial_time = vals[2]

    for i in range(100000000):
        pass

        if i > 0:
            # print_tuple([time.clock()], new_line=False, precision=4)
            # print('', end='')
            # print_tuple([after - before], new_line=False, precision=5)
            # print('', end='')
            print(input_str, end=' ')  # torque
            print_tuple([(vals[3] - last_vals[3])/(vals.time - last_vals.time)], new_line=False)  # upper velocity
            print('', end='')
            print_tuple([vals[3]], new_line=False)  # upper velocity
            print('', end='')
            print_tuple([vals[1]], new_line=False)  # upper angle
            print('', end='')
            # print_tuple([vals[2]], new_line=False)  # lower velocity
            # print('', end='')
            # print_tuple([vals[0]], new_line=False)  # lower angle
            # print('', end='')
            # print_tuple([(gyro_vals[0] + 28) * 0.001070423], new_line=False)

            print()  # new line
            print(end='')

        try:
            last_vals = vals
            last_torque = input_str
        except UnboundLocalError: pass

        # input_str = ((i / 2 - 1) % 2 - 1 )  * 0.25
        input_str = 4 - i
        if input_str % 2 == 1:
            input_str *= -1
        before = time.clock()
        # vals = a.servo_loop(input_str)
        vals = a.write_read(input_str)
        after = time.clock()
        serial_time = vals[2]
        print(input_str)
        time.sleep(max(dt - (after - before), 0))

    a.write_read(0)
    # time.sleep(1000000)

def test_serial_errors():
    n_errors = 0
    ticks = 0
    while not exit:
        response = a.ping()
        if response.hex() != '12':
            n_errors += 1
            print("didn't recieve 0x12 at tick", ticks, ", actually relieved", response.hex(),
                  ",", n_errors, "errors so far")
        ticks += 1


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


def print_tuple(tup: (), new_line: bool = False, precision: int = 6) -> None:
    """
    Neatly prints tuples to the specified precision (in decimal places).
    :param new_line: If true then a line break will be printed after the passed tuple
    :param tup: The iterable to print
    :param precision: The number of decimal places to print
    :return: None
    """
    print_tuple_helper(tup, precision)
    if new_line:
        print()


def print_tuple_helper(tup: (), precision: int) -> None:
    if type(tup).__name__ == 'tuple' or type(
            tup).__name__ == 'list' or type(tup).__name__ == 'ndarray':
        for i in tup:
            if type(i).__name__ == 'tupple' or type(
                    i).__name__ == 'list' or type(i).__name__ == 'ndarray':
                print('[', end=' ')
                print_tuple_helper(i, precision),
                print(']', end=' ')
            else:
                if i < 0:
                    precision -= 1
                print((('%.' + str(precision) + 'f') % float(i) + ""), end=' ')
    else:
        print((str(tup) + str(precision) + 'f'), end=' ')


def sub(a, b):
    """
    Subtracts b from a and returns result. 
    :param a: 
    :param b: 
    :return: 
    """
    return a - b

if __name__ == '__main__':
    main()
