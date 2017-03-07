"""
Traverses a lookup table trying to get to a supplied endpoint. When run it will load a numpy array (table_[shape].npy)
"""
__docformat__ = 'restructuredtext en'

import math
import time
from os import listdir
import re  # regular expression module (used in table file name retrieval)
import ast  # used for tuple parsing in table loading
import types
from builtins import min
import traceback

import numpy

# import visual_hopper
import lookup_table_generator

# import lookup_table_hopper_helper as helper  # imported after debug flags are checked

table_id_number = 301

# NOTE: these numbers will be replaced if a table_id_number is used. 
# table 15
q1_high = math.pi * 2
q1_low = 0.0
q2_high = math.pi
q2_low = -1.0 * math.pi
q1_vel_high = 5.0
q1_vel_low = -5.0
q2_vel_high = 7.0
q2_vel_low = -7.0
torque_high = 3
torque_low = -3

resolution = .2  # resolution of the lookup table in radians
velocity_resolution = .2
torque_resolution = 1  # resolution to step torques by
fps = None
q1_friction = 0
q2_friction = 0
# resolution values are unused because they are easily determined by the
# table dimensions

l1 = .5
l2 = .75
m1 = 1
m2 = 1

g = -9.81

weights = [0.0,  # shoulder
           0.0,  # elbow
           0.0,  # shoulder velocity
           0.0,  # elbow velocity
           0.0,  # torque
           10.0,  # center of gravity
           10.0,  # future center of gravity (approximated)
           0.0,  # kinetic energy
           1.0]  # angular momentum

use_ode_as_lookup_table = False

# global debug flags
print_on = True
verbose = True
min_dist_debug = 99999  # independent counter for debugging
use_cython = True  # use cython for interpolation (turn off when cython isn't installed (like on windows))

if use_cython:
    import lookup_table_hopper_helper as helper

table = None


def main(name):
    import sys
    sys.exit()
    # This method should no longer be used, it doesn't do anything useful and probably has many bugs

    global table
    table = numpy.load(name)
    print('table shape:', table.shape)

    startpoint_index = get_index_interpolated([math.pi / 2.0, 0.0, 0.0, 0.0])
    print(startpoint_index)
    targetpoint_index = get_index_interpolated([math.pi / 2.0, 0.0, 0.0, 0.0])

    print('target = ', targetpoint_index)

    # dist, location_index = search(startpoint_index, targetpoint_index, 3, 3, False)
    time_before = time.time()
    try:
        dist, location_angles, path = start_search_single_step(startpoint_index, targetpoint_index, 5, 3, 7)
        time_after = time.time()
        print()
        print()
        print('ended up', dist, 'away from target at :', location_angles, 'target:', end=' ')
        print(get_angles_no_torque(targetpoint_index))
        print('starting distance:', end=' ')
        print(dist_between(get_angles_no_torque(targetpoint_index), get_angles_no_torque(startpoint_index)))
        print('path:', end=' ')
        print(' '.join(map(str, path)))
        print()

        print('abbreviated path (actual taken)')
        for i in range(0, len(path)):
            print('[', end=' ')
            print_tupple(path[i][0], new_line=False)
            print(']', end=' ')
        print()

        print('time elapsed =', time_after - time_before)

        print('torques taken', end=' ')
        for i in range(0, len(path)):
            if not isinstance(path[i][0], tuple):
                # for j in range(0, len(path[i])):
                if isinstance(path[i][0], list) and len(path[i][0]) == 5:
                    print(path[i][0][4], end=' ')
                    # visual_hopper.display(path)

    except ValueError:
        print('simulation cut off by exception')


def initialize(name, imported_table=None):
    """
    Initialized this module for external use. After calling

    Args:
        name (str): File name of the lookup table to use.
    """
    global table
    if imported_table is None:
        try:
            table = numpy.load(name)
        except:
            # TODO: make this less broad
            table = numpy.load(get_table_name(name))
    else:
        table = imported_table


def set_parameters(q1_high_=math.pi * 2, q1_low_=0.0, q2_high_=math.pi, q2_low_=-1.0 * math.pi, q1_vel_high_=5.0,
                   q1_vel_low_=-5.0, q2_vel_high_=7.0, q2_vel_low_=-7.0, torque_high_=3, torque_low_=-3, resolution_=.2,
                   velocity_resolution_=.2, torque_resolution_=1):
    global q1_high, q1_low, q2_high, q2_low, q1_vel_high, q1_vel_low, q2_vel_high, q2_vel_low, torque_high, torque_low
    global resolution, velocity_resolution, torque_resolution
    q1_high = q1_high_
    q1_low = q1_low_
    q2_high = q2_high_
    q2_low = q2_low_
    q1_vel_high = q1_vel_high_
    q1_vel_low = q1_vel_low_
    q2_vel_high = q2_vel_high_
    q2_vel_low = q2_vel_low_
    torque_high = torque_high_
    torque_low = torque_low_

    resolution = resolution_
    velocity_resolution = velocity_resolution_
    torque_resolution = torque_resolution_


def get_next_torque(start_angles, target_angles, levels, n_torques, n_iterations, has_bounds=False):
    """
    Calculates the ideal next torque to be taken.
    Args:
        start_angles (float, float, float, float): Starting/current location
            of the simulated acrobot.
        target_angles (float, float, float, float): Desired ending position
            of the simulation acrobot.
        levels (int): the number of levels to look down
        n_torques (int): the approximate number of torques to use (no guarantee
            are made about the actual number tried)
    Returns: 
        int: the torque that should be applied
        tuple [[float, float, float, float, float],...]: the path taken
        tuple [float, float]: starting distance and distance after the first step. 
    """
    #  = get_next_angles_interpolated(get_index_interpolated([1.51105, 1.61649, -0.09974, 0.34533]) + [get_torque_index(50)])
    _, _, path = search(get_index_interpolated(start_angles), 0, get_index_interpolated(
        target_angles), levels, levels, n_torques, n_iterations=n_iterations)
    assert len(path) > 0
    if print_on:
        print('tree returned path', end=' ')
        print_tupple(path, new_line=True, precision=2)
    if len(path) <= 1:
        raise Exception("can't get any closer than current state")
    return path[0][4], path, [dist_between(start_angles, target_angles, torque=0),
                              dist_between(path[1], target_angles, torque=path[0][4])]


def start_search_single_step(
        startpoint_index, target_index, repetitions, layers, n_torques):
    """
    Acts as a wrapper for search() allowing for multiple iterations 
    (choosing best torque, advancing, then trying again)
    Args:
        tuple [float, float, float, flaot]: start point of acrobot as a table index (not angles)
        tuple [float, float, float, flaot]: target point of acrobot as a table index (not angles)
        int: number of repetitions to perform
        int: number of steps ahead to look before choosing a torque (2 or 3 normally works fine)
        int: number of torques to consider at each step (10 normaly works fine)
    Returns:
        float: final distance from the target position
        tuple [float, float, float, float]: the final location of the acrobot in angles 
            (not table indices)
        tuple [[float, float, float, float, float], [float, float, float, float, float],...]]: all
            previous acrobot positions (angles not indices)
    """
    current_location_angles = None
    current_location_index = startpoint_index
    total_path = list()
    for _i in range(0, repetitions):

        if print_on:
            print()
            print()
            print('starting new itteration (new tree)')
            print()

        _dist, _location_index, path = search(current_location_index, 0, target_index, layers, layers, n_torques)

        print(get_angles_no_torque(_location_index))
        current_location_angles = get_angles_no_torque(_location_index)

        if len(path) == 0:
            if print_on:
                print('stopping simulation early, forced out of table bounds')
            return dist_between(
                current_location_angles, get_angles_no_torque(target_index),
                torque=0), current_location_angles, total_path

        torque_applied = path[0][4]
        total_path.append(path)

        # set current_location_index to correct position for next iteration
        current_location_index = get_index_interpolated(path[1])

        if print_on:
            print()
            print('finnished tree, moving to ', end=' ')
            print_tupple(current_location_angles, new_line=False)
            print(' on torque ' + str(torque_applied))

    return dist_between(current_location_angles, get_angles_no_torque(
        target_index), torque=0), current_location_angles, total_path


# TODO: fix non-linear torque range
def search(start_index, torque_used, target_index, recursive_level, initial_level,
           n_torques, n_iterations=1):
    """
    Searches the passed lookup table for the optimal path to the passed target.
    :rtype : float, tupl, tupl
    Args:
        table (numpy array): lookup table
        start_index (float, float, float, float): Tuple containing the
            lookup table index location of the acrobot.
        torque_used (float): the torque applied that got the acrobot
            to the current position.
        target_index (float, float, float, float): the target table index.
        recursive_level (int): The number of levels left to search. Any
            particular search branch ends when the level becomes 0.
        initial_level (int): the total depth of the search tree.
        n_torques (int): the number of torques to apply. +- 2 torque values
            may actually be used.
        has_bounds (boolean): can the elbow and shoulder swing in a full circle
            or are they artificially constrained (by a table for instance)

    Returns:
        float: closest distance attained
        tupl: Index location of the closest distance attained.
        tupl: The path used to get to the returned closest index. Torques are contained in the returned path.
    """

    # No more levels to look down
    if recursive_level < 1:
        distance = dist_between(
            get_angles_no_torque(start_index), get_angles_no_torque(target_index), torque=torque_used)
        if print_on:
            global min_dist_debug
            min_dist_debug = min([min_dist_debug, distance])
            if verbose:
                print(' -- stopped at position ', end=' ')
                print_tupple(
                    get_angles_no_torque(start_index),
                    new_line=False)
                print(' with distance ' + str('{0:.4f}'.format(distance)), end=' ')
                print('to target: ', get_angles_no_torque(target_index))
        # returning distance then address
        return distance, start_index, [get_angles_no_torque(start_index)]

    # Still more levels to look down
    else:
        closest_dist = 992
        closest_address_index = start_index
        closest_path = list()
        closest_torque_index = get_torque_index(0)

        for iteration in range(0, n_iterations):
            # debugging tracing code
            if print_on:
                print()
                for _i in range(0, initial_level - recursive_level):
                    print('\t', end=' ')
                print('  starting new level ' + str(recursive_level), end=' ')
                # print_tuple(recursive_level, new_line = False),
                print((' at  '), end=' ')
                print_tupple(get_angles_no_torque(start_index), new_line=False)
                print(' with distance ', dist_between(get_angles_no_torque(start_index),
                                                      get_angles_no_torque(target_index), torque_used))

            # step = (torque_high - torque_low) / (2 * math.pow(n_torques, iteration))
            # torque_vals = numpy.linspace(closest_torque_index - step, closest_torque_index + step, n_torques)
            torque_vals = numpy.linspace(torque_low, torque_high, n_torques)
            for torque_val in torque_vals:
                torque_index = get_torque_index(torque_val)
                # debugging tracing code
                if print_on and verbose:
                    for _i in range(0, initial_level - recursive_level):
                        print("\t  ", end=' ')
                    print(' applying torque: ', end=' ')
                    print("%.2f" % torque_val, end=' ')
                    print(' from level ' + str(recursive_level), end=' ')

                try:
                    if use_cython:
                        next_index = get_index_interpolated(helper.get_next_angles_interpolated(table, start_index +
                                                                                                [torque_index]))
                    else:
                        next_index = get_index_interpolated(get_next_angles_interpolated(start_index + [torque_index]))

                    # if it is outside of the lookup table
                    if not is_within_bounds(next_index):
                        closest_dist_tmp = 9999
                        if print_on:
                            if next_index == [880.0000, 880.0000, 880.0000, 880.0000]:
                                print("not enough data filled in to do interpolation")
                            else:
                                print("out of bounds data returned:", end=' ')
                                print_tupple(get_angles(next_index), precision=4, new_line=True)
                    else:
                        returns = search(next_index, torque_val, target_index, recursive_level - 1, initial_level,
                                         n_torques, n_iterations=n_iterations)
                        if (type(returns) is list or type(returns) is tuple) or len(returns) == 3:
                            closest_dist_tmp, closest_address_index_tmp, path = returns
                        else:
                            print("something went wrong with search (", returns, ")")

                    if closest_dist_tmp < closest_dist and len(path) > 0 and (
                                    path[0][0] != 9999.0 and path[0][0] != 8888.0):
                        closest_dist = closest_dist_tmp
                        closest_address_index = closest_address_index_tmp
                        closest_torque_index = torque_index
                        closest_path = path

                except ValueError:
                    if True:
                        for _i in range(0, initial_level - recursive_level):
                            print('\t', end=' ')
                        print('index interpolation failed, aborting current branch. ')
                        print_tupple(start_index + [torque_index], new_line=True)
                        print(traceback.format_exc())
                except Exception as e:
                    print(e)

        # success, return closest path
        if closest_torque_index != -1:
            # final_dist = closest_dist + dist_between(get_angles_no_torque(start_index),
            #                                          get_angles_no_torque(target_index), torque_used)
            final_dist = closest_dist
            if print_on:
                for _i in range(0, initial_level - recursive_level):
                    print('\t', end=' ')
                print('  choosing torque:', str(get_torque_val(closest_torque_index)), 'with distance', \
                      str('{0:.4f}'.format(final_dist)), '(ending distance:', closest_dist, ')')

            return final_dist, closest_address_index, [
                get_angles_no_torque(start_index) + [map_to(closest_torque_index, 0, table.shape[4] - 1,
                                                            torque_low, torque_high)]] + closest_path

        # all interpolation results failed, don't add to path and return starting location as closest address
        else:
            if print_on:
                valid_starting = is_within_bounds(closest_address_index)
                for _i in range(0, initial_level - recursive_level):
                    print('\t', end=' ')
                print('all interpolation attempts failed, not advancing path')
                for _i in range(0, initial_level - recursive_level):
                    print("\t", end=' ')
                print('starting position was inside table bounds: ' + str(valid_starting))
            return closest_dist, closest_address_index, closest_path


def contains_negatives(values):
    """Returns true if the passed tupple contains no negative values and false otherwise"""
    for number in values:
        if number < 0:
            return True
    return False


def dist_between(a, b, torque=0, weight_set=0):
    """
    Returns a calculated distance between the two passed acrobot states.
    :return: distance between the two passed states
    :rtype : float
    Args:
        a (float, float, float, float): the current acrobot position
        b (float, float, float, float): the target acrobot position
        torque float: the torque applied. Defaults to 0
    """
    weightings = weights
    if weight_set == 1:  # weights used in diagnostics
        weightings = [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]  # only considers angles and velocities, not used for balancing
    if type(weight_set) == tuple:
        weightings = weight_set

    if 5 >= len(a) >= 4 == len(b):
        total = 0
        for i in range(2):
            total += math.pow(abs(a[i] - b[i]) % (math.pi * 2), 1) * weightings[i]
        for i in range(2, 4):  # velocities get a separate if statement because they should not be modded by 2*pi
            total += abs(math.pow(a[i] - b[i], 1)) * weightings[i]
        # total = math.sqrt(total)
        # total += abs(torque - get_holding_torque(a)) * weightings[4]
        # target_center_of_gravity = math.copysign(math.sqrt(abs((a[1] - b[1]))), a[1] - b[1]) / 100.0 * weightings[8]
        # total += (abs(get_center_of_gravity(a, dt=weightings[5]) - target_center_of_gravity)) * weightings[6]
        # if use_cython:
        #     a_energy = helper.get_mechanical_energy(a, l1, l2, m1, m2, g)
        #     b_energy = helper.get_mechanical_energy(b, l1, l2, m1, m2, g)
        # else:
        #     a_energy = get_mechanical_energy(a)
        #     b_energy = get_mechanical_energy(b)
        # total += abs(a_energy - b_energy) * weightings[7]
        return total


def is_within_bounds(tup):
    shape = table.shape
    for i in range(0, len(tup)):
        if tup[i] >= shape[i] or tup[i] < 0:
            return False
    return True


def is_within_set_bounds(tupl):
    if 0 < tupl[0] < math.pi and -130 * math.pi / 180 < tupl[1] < 130 * math.pi / 180:
        return True
    else:
        return False


# returns the acrobot state at the given tuple (not interpolated)
def get_next_angles(tupl):
    answer = table
    for i in tupl:
        answer = answer[i]
    return answer


# Make global to module so that get_next_angles_interpolated doesn't have to alocate every time it runs.
sums2 = numpy.zeros((2, 2, 2, 2, 2, 1))


def get_next_angles_interpolated(tupl):
    """Gets and returns an interpolated set of angles from a table index. Extrapolation will be performed if the passed
    index is outside of the loaded table. It should be passed lookup table indices, and it will return angles.
    :param tupl: [float, float, float, float, float]
    :return: [float, float, float, float]
    """

    tupl0 = float(tupl[0])
    tupl1 = float(tupl[1])
    tupl2 = float(tupl[2])
    tupl3 = float(tupl[3])
    tupl4 = float(tupl[4])
    # q1a, q1b, q2a, q2b, v1a, v1b, v2a, v2b, ta, tb = 0
    q1a = math.floor(tupl0)
    q2a = math.floor(tupl1)

    v1a = math.floor(tupl2)
    v2a = math.floor(tupl3)
    ta = math.floor(tupl4)

    q1a = fixbounds(q1a, 0, table.shape[0] - 1)
    q2a = fixbounds(q2a, 0, table.shape[1] - 1)
    v1a = fixbounds(v1a, 0, table.shape[2] - 1)
    v2a = fixbounds(v2a, 0, table.shape[3] - 1)
    ta = fixbounds(ta, 0, table.shape[4] - 1)

    q1b = q1a + 1
    q2b = q2a + 1
    v1b = v1a + 1
    v2b = v2a + 1
    tb = ta + 1

    # take a cube out of the larger table surrounding the point in question
    i, q1, q2, v1, v2, t, t1, t2, flip_temp = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    mini = numpy.zeros([2, 2, 2, 2, 2, 4], dtype=numpy.float32)
    mini2 = numpy.zeros([2, 2, 2, 2, 2, 4], dtype=numpy.float32)
    for a in range(2):
        for b in range(2):
            for c in range(2):
                for d in range(2):
                    for e in range(2):
                        for f in range(4):
                            mini2[a, b, c, d, e, f] = table[q1a + a, q2a + b, v1a + c, v2a + d, ta + e, f]

    # now interpolate within the mini array
    output = list()

    # interpolate first 4 dimensions getting rid of dimension 5
    for i in range(4):
        for q1 in range(0, 2):
            for q2 in range(0, 2):
                for v1 in range(0, 2):
                    for v2 in range(0, 2):
                        if -0.0001 < table[q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta + 1, i] < 0.0001:
                            mini[q1][q2][v1][v2][0][i] = table[q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta, i]
                            # print "first"
                            # return [999, 999, 999 ,6667]
                        elif -0.0001 < table[q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta, i] < 0.0001:
                            mini[q1][q2][v1][v2][0][i] = table[q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta + 1, i]
                            # print "second"
                            # return [990, 990, 990 ,6666]
                        else:
                            mini[q1][q2][v1][v2][0][i] = map_to(tupl4, ta, tb,
                                                                table[q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta, i],
                                                                table[
                                                                    q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta + 1, i])

                        if mini[q1][q2][v1][v2][0][i] > 100.0:
                            print("!!!!!!")
                            print(mini[q1][q2][v1][v2][0][i])
                            print(q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta, i)

        # interpolate dimension 4
        for q1 in range(0, 2):
            for q2 in range(0, 2):
                for v1 in range(0, 2):
                    if -0.000001 < mini[q1][q2][v1][0][0][i] < 0.000001:
                        mini[q1][q2][v1][0][0][i] = mini[q1][q2][v1][1][0][i]
                    elif -0.000001 < mini[q1][q2][v1][1][0][i] < 0.000001:
                        mini[q1][q2][v1][0][0][i] = mini[q1][q2][v1][0][0][i]
                    else:
                        mini[q1][q2][v1][0][0][i] = map_to(tupl3, v2a, v2b, mini[q1][q2][v1][0][0][i],
                                                           mini[q1][q2][v1][1][0][i])
                    if mini[q1][q2][v1][0][0][i] > 100.0:
                        print("!!!!!!")
                        print(mini[q1][q2][v1][v2][0][i])
                        print(q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta, i)

        # interpolate dimension 3
        for q1 in range(0, 2):
            for q2 in range(0, 2):
                if -0.000001 < mini[q1][q2][0][0][0][i] < 0.000001:
                    mini[q1][q2][0][0][0][i] = mini[q1][q2][1][0][0][i]
                elif -0.000001 < mini[q1][q2][1][0][0][i] < 0.000001:
                    mini[q1][q2][0][0][0][i] = mini[q1][q2][0][0][0][i]
                else:
                    mini[q1][q2][0][0][0][i] = map_to(tupl2, v1a, v1b, mini[q1][q2][0][0][0][i],
                                                      mini[q1][q2][1][0][0][i])
                if mini[q1][q2][0][0][0][i] > 100.0:
                    print("!!!!!!")
                    print(mini[q1][q2][v1][v2][0][i])
                    print(q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta, i)

        # interpolate dimension 2
        for q1 in range(0, 2):
            if -0.000001 < mini[q1][0][0][0][0][i] < 0.000001:
                mini[q1][0][0][0][0][i] = mini[q1][1][0][0][0][i]
            elif -0.000001 < mini[q1][1][0][0][0][i] < 0.000001:
                mini[q1][0][0][0][0][i] = mini[q1][0][0][0][0][i]
            else:
                mini[q1][0][0][0][0][i] = map_to(tupl1, q2a, q2b, mini[q1][0][0][0][0][i], mini[q1][1][0][0][0][i])
            if mini[q1][0][0][0][0][i] > 100.0:
                print("!!!!!!")
                print(mini[q1][q2][v1][v2][0][i])
                print(q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta, i)

        # interpolate only remaining dimension and append to output
        if -0.000001 < mini[0][0][0][0][0][i] < 0.000001:
            result = mini[1][0][0][0][0][i]
        elif -0.000001 < mini[1][0][0][0][0][i] < 0.000001:
            result = mini[0][0][0][0][0][i]
        else:
            result = map_to(tupl0, q1a, q1b, mini[0][0][0][0][0][i], mini[1][0][0][0][0][i])

        if len(output) > 0 and output[len(output) - 1] > 10.0:
            print("!!!!!!")
            print(mini[q1][q2][v1][v2][0][i])
            print(q1a + q1, q2a + q2, v1a + v1, v2a + v2, ta, i)

        if i == 0 and -0.0001 < result < 0.0001:
            # print "not enough data to do interpolation (all zeros)"
            # print adjust[0], adjust[1], adjust[2], adjust[3], adjust[4]
            return [880.0, 880.0, 880.0, 880.0]
        else:
            output.append(result)

    return output


def fixbounds(a, min_val, max_val):
    if a < min_val:
        a = min_val
    if a > max_val - 1:
        a = max_val - 1
    return a


def get_next_angles_interpolated_old(tupl):
    """Gets and returns an interpolated set of angles from a table index. Extrapolation will be performed if the passed
    index is outside of the loaded table. It should be passed lookup table indices, and it will return angles.

    This function is kept here so that the cython version can be checked against it.

    :param tupl: [float, float, float, float, float]
    :return: [float, float, float, float]
    """

    if False:  # use simulation instead fo lookup table
        angles = get_angles(tupl)
        return lookup_table_generator.run_simulation(*angles, run_time=1.0 / 15, fps=150)

    # nearest neighbor interpolation (almost as cutting edge mathmatics as turbo encabulators)
    if False:
        return list(table[int(round(tupl[0])), int(round(tupl[1])), int(round(tupl[2])), int(round(tupl[3])), int(
            round(tupl[4]))])

    q1a = int(math.floor(tupl[0]))
    q1b = int(math.ceil(tupl[0]))
    if q1a == q1b:
        q1b += 1

    q2a = int(math.floor(tupl[1]))
    q2b = int(math.ceil(tupl[1]))
    if q2a == q2b:
        q2b += 1

    v1a = int(math.floor(tupl[2]))
    v1b = int(math.ceil(tupl[2]))
    if v1a == v1b:
        v1b += 1

    v2a = int(math.floor(tupl[3]))
    v2b = int(math.ceil(tupl[3]))
    if v2a == v2b:
        v2b += 1

    ta = int(math.floor(tupl[4]))
    tb = int(math.ceil(tupl[4]))
    if ta == tb:
        tb += 1

    q1a, q1b = fix_bounds(q1a, q1b, 0, table.shape[0] - 1)
    q2a, q2b = fix_bounds(q2a, q2b, 0, table.shape[1] - 1)
    v1a, v1b = fix_bounds(v1a, v1b, 0, table.shape[2] - 1)
    v2a, v2b = fix_bounds(v2a, v2b, 0, table.shape[3] - 1)
    ta, tb = fix_bounds(ta, tb, 0, table.shape[4] - 1)

    # take a cube out of the larger table surounding the point in question
    grid = numpy.ix_([q1a, q1b], [q2a, q2b], [v1a, v1b], [v2a, v2b], [ta, tb])
    mini = table[grid]

    # now interpolate within the mini array
    mini_separated = numpy.split(mini, 4, axis=5)
    output = list()

    for mini_1 in mini_separated:
        q1r = tupl[0] - q1a
        q2r = tupl[1] - q2a
        v1r = tupl[2] - v1a
        v2r = tupl[3] - v2a
        tr = tupl[4] - ta

        mini_2 = mini_1[::-1, ::-1, ::-1, ::-1, ::-1]

        vals = [[q1r, 1 - q1r], [q2r, 1 - q2r], [v1r, 1 - v1r], [v2r, 1 - v2r], [tr, 1 - tr]]
        for a in range(0, 2):
            for b in range(0, 2):
                for c in range(0, 2):
                    for d in range(0, 2):
                        for e in range(0, 2):
                            # noinspection PyTypeChecker
                            sums2[a][b][c][d][e][0] = vals[0][a] * vals[1][b] * vals[2][c] * vals[3][d] * vals[4][e]
        output.append(numpy.sum(numpy.multiply(sums2, mini_2)))
    return output


def get_mechanical_energy(tupl):
    """
    Returns the potential energy of the acrobot at the passed state. Throws an exception if a tuple without 4 or 5
    elements is passed in.
    :param tupl: The acrobot's current state (4 or 5 tuples are okay)
    :return: potential energy of the acrobot
    :rtype : float
    """

    if len(tupl) == 4:
        q1, q2, w1, w2 = tupl
    elif len(tupl) == 5:
        q1, q2, w1, w2, t = tupl
    else:
        raise Exception('get_mechanical_energy received tuple of len ' + str(tupl))

    # potential energy
    m1y = math.sin(q1) * l1
    m2y = math.sin(q1 + q2) * l2

    pe1 = m1y * m1 * g * -1  # -1 because gravity is also negative and PE increase as m1x increases
    pe2 = (m2y + m1y) * m2 * g * -1

    v1 = w1 * l1
    v2 = w2 * l2

    v1x = math.sin(q1) * v1 * -1
    v1y = math.cos(q1) * v1

    p1 = numpy.array((math.cos(q1) * l1, math.sin(q1) * l1))
    p2_rel = numpy.array((math.cos(q1 + q2) * l2, math.sin(q1 + q2) * l2))
    p2 = p1 + p2_rel
    d = math.sqrt(p2[0] ** 2 + p2[1] ** 2)
    theta_d = math.acos(p2[0] / d)
    if p2[1] < 0:
        theta_d = math.pi * 2 - theta_d

    v2x = (v2 * math.sin(q1 + q2) + d * w1 * math.sin(theta_d)) * -1
    v2y = v2 * math.cos(q1 + q2) + d * w1 * math.cos(theta_d)

    ke = .5 * m1 * (v1x ** 2 + v1y ** 2) + .5 * m2 * (v2x ** 2 + v2y ** 2)

    return pe1 + pe2 + ke


def get_angular_momentum(state):
    """
    Returns the angular momentum of the acrobot around the origin. An exception will be thrown if a tuple without 4 or 5
    elements is passed in.
    **NOTE: THIS FUNCTION HAS NOT BEEN THOROUGHLY TESTED**
    :param state: The acrobot's current state (4 or 5 tuples are okay)
    :return: angular momentum of acrobot
    :rtype: float
    """
    if len(state) == 4:
        q1, q2, w1, w2 = state
    elif len(state) == 5:
        q1, q2, w1, w2, t = state
    else:
        raise Exception('get_mechanical_energy received tuple of len ' + str(state))

    v1 = w1 * l1
    v2 = w2 * l2

    v1x = math.sin(q1) * v1 * -1
    v1y = math.cos(q1) * v1

    p1 = numpy.array((math.cos(q1) * l1, math.sin(q1) * l1))
    p2_rel = numpy.array((math.cos(q1 + q2) * l2, math.sin(q1 + q2) * l2))
    p2 = p1 + p2_rel
    d = math.sqrt(p2[0] ** 2 + p2[1] ** 2)
    theta_d = math.acos(p2[0] / d)
    if p2[1] < 0:
        theta_d = math.pi * 2 - theta_d

    v2x = (v2 * math.sin(q1 + q2) + d * w1 * math.sin(theta_d)) * -1
    v2y = v2 * math.cos(q1 + q2) + d * w1 * math.cos(theta_d)

    momentum1 = l1 * m1 * v1

    v2_mat = numpy.array((v2x, v2y, 0))
    momentum2 = numpy.cross(p2, v2_mat * m2)
    momentum2 = math.copysign(numpy.linalg.norm(momentum2), momentum2[2])

    return momentum1 + momentum2


def fix_bounds(lower, upper, min_val, max_val):
    """
    Adjusts two numbers to be within lower range while keeping lower difference between them of one.
    :type lower: int
    :type upper: int
    :type min_val: int
    :type max_val: int
    :param lower: lwer value
    :param upper: upper value
    :param min_val: lower bound
    :param max_val: upper bound
    :rtype: [int, int]
    """
    if min_val != min(lower, min_val):
        lower = min_val
        upper = lower + 1
    if max_val != max(upper, max_val):
        upper = max_val
        lower = upper - 1
    return [lower, upper]


def get_next_angles_interpolated_old(tupl):
    return get_next_angles_interpolated(tupl)


# returns the acrobot state at the given tuple
def get_angles(index):
    """
    :param index: acrobot table indicies
    :return: acrobot angles (4 element tuple)
    """
    angle1 = map_to(index[0], 0.0, table.shape[0] - 1, q1_low, q1_high)
    angle2 = map_to(index[1], 0.0, table.shape[1] - 1, q2_low, q2_high)
    angle1_vel = map_to(index[2], 0.0, table.shape[2] - 1, q1_vel_low, q1_vel_high)
    angle2_vel = map_to(index[3], 0.0, table.shape[3] - 1, q2_vel_low, q2_vel_high)
    if len(index) == 5:
        return list((angle1, angle2, angle1_vel, angle2_vel,
                     map_to(index[4], 0, table.shape[4] - 1, torque_low, torque_high)))
    return list((angle1, angle2, angle1_vel, angle2_vel))


# returns the acrobot state at the given tuple
def get_angles_no_torque(index):
    """
    :rtype : [float, float, float, flaot]
    """
    angle1 = map_to(index[0], 0, table.shape[0] - 1, q1_low, q1_high)
    angle2 = map_to(index[1], 0, table.shape[1] - 1, q2_low, q2_high)
    angle1_vel = map_to(index[2], 0, table.shape[2] - 1, q1_vel_low, q1_vel_high)
    angle2_vel = map_to(index[3], 0, table.shape[3] - 1, q2_vel_low, q2_vel_high)
    return list((angle1, angle2, angle1_vel, angle2_vel))


# returns a tuple representing the acrobot's position in the lookup table
def get_index_interpolated(angles):
    """
    Converts angles to lookup table indicies.
    :param angles: [float, float, float, float]
    :return: [float, float, float, float]
    """
    angle1 = map_to(angles[0], q1_low, q1_high, 0, table.shape[0] - 1)
    angle2 = map_to(angles[1], q2_low, q2_high, 0, table.shape[1] - 1)
    angle1_vel = map_to(angles[2], q1_vel_low, q1_vel_high, 0, table.shape[2] - 1)
    angle2_vel = map_to(angles[3], q2_vel_low, q2_vel_high, 0, table.shape[3] - 1)
    if len(angles) == 5:
        return list(
            (angle1, angle2, angle1_vel, angle2_vel,
             map_to(angles[4], torque_low, torque_high, 0, table.shape[4] - 1)))
    return (angle1, angle2, angle1_vel, angle2_vel)


def get_torque_val(torque_index):
    """
    Converts a torque table index to a value.
    :rtype: float
    :type torque_index: float
    """
    return map_to(torque_index, 0, table.shape[4] - 1, torque_low, torque_high)


def get_torque_index(torque_val):
    """ :rtype: float """
    return map_to(torque_val, torque_low, torque_high, 0, table.shape[4] - 1)


def map_to(x, in_min, in_max, out_min, out_max):
    """Translates a value from one domain to another keeping proportions constant
    :type x: float
    :type in_min: float
    :type in_max: float
    :type out_min: float
    :type out_max: float
    :rtype : float
    """
    if in_max == in_min:
        raise ZeroDivisionError
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def get_center_of_gravity(tup, dt=0.0):
    """
    Returns x coridinate of the center of gravity of the acrobot.
    Args:
        tup (float,float,float,float): current state
    Returns:
        float: x coordinate of the center of gravity
    """

    m1x = l1 * m1 * math.cos(tup[0] + tup[2] * dt)
    m2x = m2 * (l1 * math.cos(tup[0] + tup[2] * dt) +
                l2 * math.cos(tup[0] + tup[2] * dt + tup[1] + tup[3] * dt))
    return m1x + m2x


def get_holding_torque(tupl):
    """
    Returns the torque required to counteract gravity on the upper segment. 
    Args:
        tupl (float, float, float, float): acrobot position (angles)
    Returns:
        float: torque required
    """
    if True:  # for actual hardware
        return 0
    else:
        return m2 * l2 * 9.8 * math.cos(tupl[0] + tupl[1])


def frange(x, y, step):
    """works just like range except that floats are allowed"""
    while x < y:
        yield x
        x += step


def exponential_torque_range(start_index, n_torques, initial_level, recursive_level):
    holding_torque = get_holding_torque(get_angles_no_torque(start_index))
    if print_on:
        for _i in range(0, initial_level - recursive_level):
            print('\t', end=' ')
        print('holding torque: ', holding_torque)
    torque_vals = list()
    x_range = ((torque_high - torque_low) / 2.0) ** 4.0 ** -1
    torque_range_resolution = ((torque_high - torque_low) ** (4.0 ** -1)) / float(n_torques / 2)
    for t in frange(x_range * -1, x_range + 1, torque_range_resolution):
        torque_vals.append(
            math.copysign(abs(t) ** 4.0, t) + holding_torque)
    return torque_vals


def get_fps():
    return fps


def get_q1_friction():
    """
    :return: q1_friction
    :rtype: float
    """
    return q1_friction


def get_q2_friction():
    """
    :return: q2_friction
    :rtype: float
    """
    return q2_friction


def get_table_name(table_number):
    """
    finds the lookup table with the id number passed and changes the table 
    parameters (stored at top of this file) to match the loaded one. 
    Args:
        int: table id number
    Returns:
        String: file name of lookup table
    """
    # get the lookup table's file name from its number
    files = list(listdir('./'))
    # store matches in list to account for possibility of multiple matches
    matches = list()
    for name in files:
        if re.search('table' + str(table_number), name) is not None:
            matches.append(name)
    if len(matches) > 1:
        print('multiple tables with id ' + str(table_number) + ' found, using the last one')
    if len(matches) == 0:
        raise Exception('no table with id ' + str(table_number) + ' found')
    name = matches[len(matches) - 1]
    # change table parameters to match the loaded one
    dimensions = ast.literal_eval(name.split('_')[1].split('.n')[0])

    global q1_high, q1_low, q2_high, q2_low, q1_vel_high, q1_vel_low, q2_vel_high
    global q2_vel_low, torque_high, torque_low, resolution, velocity_resolution, torque_resolution, fps

    q1_high = dimensions[1]
    q1_low = dimensions[0]
    q2_high = dimensions[3]
    q2_low = dimensions[2]
    q1_vel_high = dimensions[5]
    q1_vel_low = dimensions[4]
    q2_vel_high = dimensions[7]
    q2_vel_low = dimensions[6]
    torque_high = dimensions[9]
    torque_low = dimensions[8]

    resolution = dimensions[10]
    velocity_resolution = dimensions[11]
    torque_resolution = dimensions[12]
    if len(dimensions) > 13:
        fps = dimensions[13]
    elif len(dimensions) > 15:
        q1_friction = dimensions[14]
        q2_friction = dimensions[15]
    return name


def get_gravity():
    """
    Returns the constant being used for gravity. Default value is -9.81 unless otherwise specified by the lookup table.
    :return: gravity (in units length per second^2). Defaults to whatever the lookup table is set to.
    :rtype : float
    """
    return g


def set_gravity(gravity):
    """
    Sets the constant used as gravity. The positive direction is up (y+ direction).
    :param g:
    :rtype : None
    """
    global g
    g = gravity


def set_weights(new_weights):
    """
    Sets the weight set used by the function dist_between()
    :param new_weights: weight set to use
    :rtype: None
    """
    global weights
    weights = new_weights


# prints nested tupples with better formatting
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
                print((('%.' + str(precision) + 'f') % i + ","), end=' ')
    else:
        print((str(tup) + str(precision) + 'f'), end=' ')


def use_ode_as_table(val):
    global use_ode_as_lookup_table
    use_ode_as_lookup_table = val


def set_debug_print(state=True):
    global print_on, verbose
    print_on = state
    verbose = state


if __name__ == '__main__':
    main(get_table_name(table_id_number))

else:
    # turn off debugging code
    # print_on = False
    # verbose = False
    print_on = True
    verbose = True
    # initialize(get_table_name(table_id_number))
