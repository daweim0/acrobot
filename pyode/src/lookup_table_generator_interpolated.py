__author__ = 'David Smart'

"""
generates a lookup table from logs writen by hardware_lookup_table_generator.py. Uses better interpolation than
nearest neighbor to generate lookup table indices from point clouds that recording acrobot movement generates.
"""

import math
from math import pi
import numpy as np
from scipy.interpolate import rbf

q1_low = 80 * pi / 180
q1_high = 100 * pi / 180
q2_low = -90 * pi / 180
q2_high = 90 * pi / 180
q1_vel_low = -1.0
q1_vel_high = 1.0
q2_vel_low = -2.0
q2_vel_high = 2.0
torque_low = -50
torque_high = 50

table_dim_low = (q1_low, q2_low, q1_vel_low, q2_vel_low, torque_low)
table_dim_high = (q1_high, q2_high, q1_vel_high, q2_vel_high, torque_high)

resolution = 0.15  # resolution of the lookup table in radians
velocity_resolution = 0.15
torque_resolution = 10  # resolution to step torques by
fps = 15
dt = 1.0 / fps
run_time = 60 * 3

output_table_number = 209


def main(log_name="log_recorder.txt", table_name=150):
    raw_data = np.ndarray((int(math.floor(q1_high / resolution)) - int(math.floor(q1_low / resolution)),
                           int(math.floor(q2_high / resolution)) - int(math.floor(q2_low / resolution)),
                           int(math.floor(q1_vel_high / velocity_resolution)) - int(
                               math.floor(q1_vel_low / velocity_resolution)),
                           int(math.floor(q2_vel_high / velocity_resolution)) - int(
                               math.floor(q2_vel_low / velocity_resolution)),
                           int(math.floor(torque_high / torque_resolution)) - int(
                               math.floor(torque_low / torque_resolution))), dtype=object)

    output_table = np.zeros((int(math.floor(q1_high / resolution)) - int(math.floor(q1_low / resolution)),
                                    int(math.floor(q2_high / resolution)) - int(math.floor(q2_low / resolution)),
                                    int(math.floor(q1_vel_high / velocity_resolution)) - int(
                                        math.floor(q1_vel_low / velocity_resolution)),
                                    int(math.floor(q2_vel_high / velocity_resolution)) - int(
                                        math.floor(q2_vel_low / velocity_resolution)),
                                    int(math.floor(torque_high / torque_resolution)) - int(
                                        math.floor(torque_low / torque_resolution)),
                                    4), dtype=np.float32)

    input_file = open(log_name, "r")
    while True:
        next_line = input_file.readline()
        next_line = next_line.replace("\n", "")
        args_raw = next_line.split(" ")
        if len(args_raw) < 2:
            next_line = input_file.readline()
            next_line = next_line.replace("\n", "")
            args_raw = next_line.split(" ")
            if len(args_raw) < 2:
                break

        # print args_raw
        args = list()
        for i in args_raw:
            args.append(i.replace(",", ""))

        # parse data from text file
        if args[0] == "(new)" or args[0] == "(old)":
            try:
                starting_angles = list()
                starting_angles.append(float(args[6].replace("[", "").replace(",", "")))
                starting_angles.append(float(args[7].replace(",", "")))
                starting_angles.append(float(args[8].replace(",", "")))
                starting_angles.append(float(args[9].replace("]", "").replace(",", "")))
                starting_angles.append(float(args[10].replace(",", "")))

                ending_angles = list()
                ending_angles.append(float(args[18].replace("[", "").replace(",", "")))
                ending_angles.append(float(args[19].replace(",", "")))
                ending_angles.append(float(args[20].replace(",", "")))
                ending_angles.append(float(args[21].replace("]", "").replace(",", "")))
                # print "  ", starting_angles, "  ", ending_angles

                add_to_table(raw_data, starting_angles, ending_angles)
            except ValueError:
                pass
            except IndexError:
                pass
        else:
            raise Exception("parsing error")

    running_total = 0.0
    n_running_total = 0
    max_stdev = 0.0
    max_index = [0, 0, 0, 0, 0]
    max_stdev_precent = 0.0

    for index in np.argwhere(raw_data):
        data = raw_data[index[0], index[1], index[2], index[3], index[4]]
        for i in range(0, 4):
            local_data = np.ndarray((len(data)), dtype=np.float32)
            for j in range(len(data)):
                local_data[j] = data[j][i]
            stdev = np.std(local_data)
            average = np.average(local_data)
            if average != 0:
                stdev_precent = stdev / average
            else:
                stdev_precent = 0

            #filter out all the outliers because they shouldn't go into the output table
            non_outliers = list()
            for i in local_data:
                if abs(i - average) <= stdev * 1.5:
                    non_outliers.append(i)
                else:
                    print("outlier found:", average, i, stdev, "index:", index, "percent = ", stdev_precent)
            non_outlier_sum = 0
            for i in non_outliers:
                non_outlier_sum += i
            try:
                output_table[index[0], index[1], index[2], index[3], index[4], i] = float(non_outlier_sum) / len(non_outliers)
            except:
                output_table[index[0], index[1], index[2], index[3], index[4], i] = np.average(local_data)

            if stdev > 0.001:
                print(index, i, stdev)

            # only include data in the average if there is more than one value
            # (stdev will be zero otherwise)
            if local_data.shape[0] > 0:
                running_total += stdev
                n_running_total += 1
                if stdev > max_stdev:
                    max_stdev = stdev
                    max_index = index
                    max_stdev_precent = stdev_precent

    print("average standard deviation = ", running_total / n_running_total)
    print("maximum standard deviation = ", max_stdev, " at ", max_index)
    print("standard deviation precentage = ", max_stdev_precent, " at ", max_index)

    # print "writing output table"
    # np.save('table' + str(output_table_number) + '_' + str([q1_low, q1_high, q2_low, q2_high, q1_vel_low,
    #                                                     q1_vel_high, q2_vel_low, q2_vel_high, torque_low,
    #                                                     torque_high, resolution, velocity_resolution,
    #                                                     torque_resolution, fps, 0, 0]), output_table)



def add_to_table(raw_data, starting_angles, ending_angles):
    index = get_index_interpolated(raw_data, starting_angles)
    index = list(map(round, index))
    # print type(raw_data[index[0], index[1], index[2], index[3], index[4]])
    if type(raw_data[index[0], index[1], index[2], index[3], index[4]]) is not list:
        raw_data[index[0], index[1], index[2], index[3], index[4]] = list()
    raw_data[index[0], index[1], index[2], index[3], index[4]].append(ending_angles)


def get_index_interpolated(table, angles):
    """
    Converts angles to lookup table indicies.
    :param angles: [float, float, float, float]
    :return: [float, float, float, float]
    """
    angle1 = map_to(angles[0], q1_low, q1_high, 0.0, table.shape[0] - 1)
    angle2 = map_to(angles[1], q2_low, q2_high, 0.0, table.shape[1] - 1)
    angle1_vel = map_to(angles[2], q1_vel_low, q1_vel_high, 0.0, table.shape[2] - 1)
    angle2_vel = map_to(angles[3], q2_vel_low, q2_vel_high, 0.0, table.shape[3] - 1)
    if len(angles) == 5:
        return list(
            (angle1, angle2, angle1_vel, angle2_vel,
             map_to(angles[4], torque_low, torque_high, 0, table.shape[4] - 1)))
    return int(round(angle1)), int(round(angle2)), int(round(angle1_vel)), int(round(angle2_vel))


def get_angles(table, index):
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


def get_torque_val(table, torque_index):
    """
    Converts a torque table index to a value.
    :rtype: float
    :type torque_index: float
    """
    return map_to(torque_index, 0, table.shape[4] - 1, torque_low, torque_high)


def get_torque_index(table, torque_val):
    """ :rtype: float """
    return map_to(torque_val, torque_low, torque_high, 0, table.shape[4] - 1)


def map_to(x, in_min, in_max, out_min, out_max):
    return (float(x) - in_min) * (float(out_max) - out_min) / (float(in_max) - in_min) + out_min


if __name__ == "__main__":
    main()
