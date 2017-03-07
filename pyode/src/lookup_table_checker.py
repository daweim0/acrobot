"""
Created on Jan 16, 2015
Checks lookup table validity by comparing lookup table and ODE output for a set of acrobot states.
@author: David Smart
"""
import math
import operator
import random
import os
import sys

import random
import matplotlib
import matplotlib.pyplot as plot
import numpy

import lookup_table_hopper as hopper
import lookup_table_generator as simulation
# import playground

hopper.initialize(204)
plot_on = True


def main(action):
    print("starting")

    max_error = 0.0
    total_error = 0
    errors = list()
    q1_s = list()
    q2_s = list()
    q1_f = list()  # for 4-dimensional mashup
    q2_f = list()
    for q1 in drange(q1_low, q1_high, (q1_high - q1_low) / n_angles):
        for q2 in drange(q2_low, q2_high, (q2_high - q2_low) / n_angles):

            if False:  # check initial velocities
                error = 0
                trials = 8
                for _ in range(0, trials):
                    q1_vel = map_to(random.random(), 0, 1, q1_vel_low, q1_vel_high)
                    q2_vel = map_to(random.random(), 0, 1, q2_vel_low, q2_vel_high)
                    # t = map_to(random.random(), 0, 1, torque_low, torque_high)
                    # q1_vel = 0
                    # q2_vel = 0
                    t = 0
                    hopper_result = (q1, q2, q1_vel, q2_vel)
                    simulation_result = simulation.run_simulation(q1, q2, q1_vel, q2_vel, t, fps=500, run_time=.04)
                    difference_temp = list(map(operator.sub, tuple(hopper_result), simulation_result))
                    difference_temp = list(map(operator.abs, difference_temp))  # differences should be absolute values
                    error += sum(difference_temp) / float(trials)

                    print("[", end=' ')
                    hopper.print_tupple(
                        [q1, q2, q1_vel, q2_vel, t], precision=3)
                    print("]    [", end=' ')
                    hopper.print_tupple(difference_temp, precision=3)
                    print("]    [", end=' ')
                    hopper.print_tupple(simulation_result, precision=3)
                    print("]")
                    if error > max_error:
                        max_error = error
                    errors.append(error)
                    q1_s.append(q1)
                    q2_s.append(q2)
                    total_error += error

            if False:  # check lookup table accuracy
                q1_vel = map_to(random.random(), 0, 1, q1_vel_low, q1_vel_high)
                q2_vel = map_to(random.random(), 0, 1, q2_vel_low, q2_vel_high)
                t = map_to(random.random(), 0, 1, torque_low, torque_high)
                hopper_result = hopper.get_next_angles_interpolated(
                    hopper.get_index_interpolated([q1, q2, q1_vel, q2_vel, t]))
                simulation_result = simulation.run_simulation(q1, q2, q1_vel, q2_vel, t)
                difference = list(map(operator.sub, tuple(hopper_result), simulation_result))
                difference = list(map(operator.abs, difference))  # differences don't need to be negative here
                print("distance =", hopper.dist_between(simulation_result, hopper_result, weight_set=1), end=' ')
                print("[", end=' ')
                hopper.print_tupple(
                    [q1, q2, q1_vel, q2_vel, t], precision=3)
                print("]    [", end=' ')
                hopper.print_tupple(difference, precision=3)
                print("]    [", end=' ')
                hopper.print_tupple(simulation_result, precision=3)
                print("]    [", end=' ')
                hopper.print_tupple(hopper_result, precision=3)
                print("]")
                print("with index", hopper.get_index_interpolated([q1, q2, q1_vel, q2_vel, t]))
                error = hopper.dist_between(hopper_result, simulation_result, torque=0, weight_set=1)
                if error > max_error:
                    max_error = error
                errors.append(error)
                q1_s.append(q1)
                q2_s.append(q2)
                total_error += error

            if False:  # draw "S curve" (where center of gravity = 0)
                difference = abs(hopper.get_center_of_gravity([q1, q2, 0, 0]))
                if difference > .25:
                    difference = .25
                difference **= 3
                print("[", end=' ')
                hopper.print_tupple(
                    [q1, q2], precision=3)
                print("]    [", end=' ')
                hopper.print_tupple(difference, precision=3)
                print("]")
                error = difference
                if error > max_error:
                    max_error = error
                errors.append(error)
                q1_s.append(q1)
                q2_s.append(q2)
                total_error += error

    if True:  # check robot generated lookup table accuracy
        torque_coloring = True
        indicies = numpy.argwhere(hopper.table)
        for index in indicies:
            if index[5] == 0 and ((not torque_coloring) or numpy.sum(hopper.table[index[0], index[1], index[2], index[3]]) > 0):
                # hopper_result = hopper.get_next_angles(index[0:5])
                # simulation_result = simulation.run_simulation(*hopper.get_angles(index[0:5]), run_time=1/10.0, fps=100)
                # difference = map(operator.sub, tuple(hopper_result), simulation_result)
                # difference = map(operator.abs, difference)  # differences don't need to be negative here
                # print "distance =", hopper.dist_between(simulation_result, hopper_result, weight_set=1),
                # print "[",
                # hopper.print_tuple(
                #     hopper.get_angles(index[0:5]), precision=3)
                # print "]    [",
                # hopper.print_tuple(difference, precision=3)
                # print "]    [",
                # hopper.print_tuple(simulation_result, precision=3)
                # print "]    [",
                # hopper.print_tuple(hopper_result, precision=3)
                # print "]    [",
                # hopper.print_tuple(index, precision=1)
                # print "]"
                # print "with index", hopper.get_index_interpolated(hopper.get_angles(index[0:5]))
                # error = hopper.dist_between(hopper_result, simulation_result, torque=0, weight_set=1)
                if torque_coloring:
                    tmp = len(numpy.where(hopper.table[index[0], index[1], index[2], index[3]] != 0)[0])
                    errors.append(tmp)
                    if tmp > max_error:
                        max_error = tmp
                else:
                    if error > max_error:
                        max_error = error
                    errors.append(error)
                angles = hopper.get_angles(index[0:5])
                if len(q2_s) == 0 or q2_s[len(q2_s)-1] != angles[1] or q1_s[len(q1_s)-1] != angles[0]:
                    q1_s.append(angles[0])
                    q2_s.append(angles[1])
                q1_f.append(angles[0]+angles[2]*.02)
                q2_f.append(angles[1]+angles[3]*.02)
                if not torque_coloring:
                    total_error += error

    print("                          [         start position          ]        [     difference     ]             [    simulation result    ]   " \
          "        [       hopper result       ]             [      index       ]")
    print(len(q1_s), len(q2_s), len(errors))
    print("maximum error:", max_error)
    print("average error =", total_error / len(errors))
    print("table shape = ", hopper.table.shape)

    step = 1
    total = len(errors)
    while len(q1_f) > step:
        for _ in range(0, step):
            error = errors.pop(0)
            x = q1_f.pop(0)
            y = q2_f.pop(0)
        plot.scatter(x, y, color=rgb_to_hex(
            (map_to(error, 0, max_error, 0, 255), 150, 0)), alpha=.5, s=15, marker='s')
        # print x, y, rgb_to_hex((map_to(error, 0, max_error, 0, 255), 150, 0)), "   ", num
    for i in range(len(q1_s)):
        plot.scatter(q1_s[i], q2_s[i], color='#4040ff', alpha=.5, s=30, marker='s')

    print("done entering data")

    plot.autoscale(enable=True, )
    plot.ylim([-120 * math.pi / 180, 120 * math.pi / 180])
    plot.xlim([110 * math.pi / 180, 70 * math.pi / 180])

    if action == "save":
        DIR = '/home/david'
        fig = matplotlib.pyplot.gcf()
        fig.set_size_inches(23.31, 12.46)
        fig.set_dpi(100)
        fig.savefig(DIR + "/" + str(len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])) + ".png")
    else:
        plot.show()


# Unlike in lookup_table_generator and lookup_table_hopper, these values don't
# need to match those of the table. It just happens to work well as a format
# for describing continuity tests (between the table and the ODE simulation).
# The module should still work if the lower bound is less than the upper bound,
# but no guarantees are made.
q1_high = math.pi * 2 - .4
q1_low = math.pi + .4
q2_high = math.pi
q2_low = -1.0 * math.pi
q1_vel_high = 5.0
q1_vel_low = -5.0
q2_vel_high = 7.0
q2_vel_low = -7.0
torque_high = 3
torque_low = -3

# number of angles, velocities, and torques to test. NOT the same as resolution
n_angles = 40.0


def drange(start, stop, step):
    r = start
    while r < stop:
        yield r
        r += step


def rgb_to_hex(rgb_tuple):
    """ convert an (R, G, B) tuple to #RRGGBB """
    hexcolor = '#%02x%02x%02x' % rgb_tuple
    # that's it! '%02x' means zero-padded, 2-digit hex values
    return hexcolor


def map_to(var, in_min, in_max, out_min, out_max):
    """Translates a value from one domain to another keeping proportions constant"""
    return (var - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == "save":
        main("save")
    else:
        main("show")
