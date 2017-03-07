"""
Generates a lookup table used by the module lookup_table_hopper.py
Created on Sep 11, 2014
@author: David Michelman
"""
import math
import numpy
import sys
import simulation
import time

q1_low = math.pi/2 - 2
q1_high = math.pi/2 + 2
q2_low = -2
q2_high = 2
q1_vel_low = -2.0
q1_vel_high = 2.0
q2_vel_low = -5.0
q2_vel_high = 5.0
torque_low = -10
torque_high = 10

resolution = 0.2  # resolution of the lookup table in radians
velocity_resolution = 0.2
torque_resolution = 1.0  # resolution to step torques by
fps = 15
q1_friction = .25
q2_friction = 1.0

check_table = False  # generates a table where each cell contains it's coordinates


def main(q1_slice):
    print("starting")

    starting_time = time.clock()
    # creates an array to store the generated lookup table. 
    table = numpy.ndarray((int(math.floor(q1_high / resolution)) - int(math.floor(q1_low / resolution)),
                           int(math.floor(q2_high / resolution)) - int(math.floor(q2_low / resolution)),
                           int(math.floor(q1_vel_high / velocity_resolution)) - int(
                               math.floor(q1_vel_low / velocity_resolution)),
                           int(math.floor(q2_vel_high / velocity_resolution)) - int(
                               math.floor(q2_vel_low / velocity_resolution)),
                           int(math.floor(torque_high / torque_resolution)) - int(
                               math.floor(torque_low / torque_resolution)),
                           4), dtype=numpy.float32)  # 4 values in state vector

    print("table dimensions:", table.shape)

    # only do one iteration if q1_slice is defined
    if q1_slice is not None:
        q2(table, q1_slice)
        # print q1_slice
    else:
        for q1_index in range(0, table.shape[0]):
            q2(table, q1_index)
            # print q1_index

    if q1_slice is not None:
        numpy.save('table43_slice_' + str(q1_slice) + "_" + str([q1_low, q1_high, q2_low, q2_high, q1_vel_low,
                                     q1_vel_high, q2_vel_low, q2_vel_high, torque_low,
                                     torque_high, resolution, velocity_resolution,
                                     torque_resolution, fps, q1_friction, q2_friction]), table)
    else:
        # write the table dimensions into the file name so that they don't have to be stored separately
        numpy.save('table42_' + str([q1_low, q1_high, q2_low, q2_high, q1_vel_low,
                                     q1_vel_high, q2_vel_low, q2_vel_high, torque_low,
                                     torque_high, resolution, velocity_resolution,
                                     torque_resolution, fps, q1_friction, q2_friction]), table)

    print(time.clock() - starting_time)

    del table


def q2(table, q1_index):
    q1_val = map_to(q1_index, 0.0, table.shape[0] - 1, q1_low, q1_high)
    for q2_index in range(0, table.shape[1]):
        q2_val = map_to(q2_index, 0.0, table.shape[1] - 1, q2_low, q2_high)

        for q1_vel_index in range(0, table.shape[2]):
            q1_vel_val = map_to(q1_vel_index, 0.0, table.shape[2] - 1, q1_vel_low, q1_vel_high)

            print("q1 range " + str((0, table.shape[0] - 1)))
            print("q2 range " + str((0, table.shape[1] - 1)))
            print("q1_vel range " + str((0, table.shape[2] - 1)))
            print("q2_vel range " + str((0, table.shape[3] - 1)))
            print("q1, q2, q1_vel, q2_vel, t")
            print(q1_index, q2_index, q1_vel_index)
            print()

            for q2_vel_index in range(0, table.shape[3]):
                q2_vel_val = map_to(q2_vel_index, 0.0, table.shape[3] - 1, q2_vel_low, q2_vel_high)

                for t_index in range(0, table.shape[4]):
                    t_val = map_to(t_index, 0.0, table.shape[4] - 1, torque_low, torque_high)
                    if not check_table:
                        state_vector = run_simulation(q1_val, q2_val, q1_vel_val, q2_vel_val, t_val,
                                                      gravity=-9.81, run_time=1.0 / fps, q1_friction_=q1_friction,
                                                      q2_friction_=q2_friction)
                        table[q1_index][q2_index][q1_vel_index][q2_vel_index][t_index][0] = state_vector[0]
                        table[q1_index][q2_index][q1_vel_index][q2_vel_index][t_index][1] = state_vector[1]
                        table[q1_index][q2_index][q1_vel_index][q2_vel_index][t_index][2] = state_vector[2]
                        table[q1_index][q2_index][q1_vel_index][q2_vel_index][t_index][3] = state_vector[3]

                    else:
                        table[q1_index][q2_index][q1_vel_index][q2_vel_index][t_index][0] = q1_val
                        table[q1_index][q2_index][q1_vel_index][q2_vel_index][t_index][1] = q2_val
                        table[q1_index][q2_index][q1_vel_index][q2_vel_index][t_index][2] = q1_vel_val
                        table[q1_index][q2_index][q1_vel_index][q2_vel_index][t_index][3] = t_val


def run_simulation(q1_val, q2_val, q1_vel_val, q2_vel_val, t_val, run_time=0.08333333, fps=150, gravity=-9.81,
                   q1_friction_=0.0, q2_friction_=0.0):
    """
    :rtype : [float, float, float, float]
    """
    return simulation.run(constant_torque=t_val, passive=True, starting_q1v=q1_vel_val,
                          starting_q2v=q2_vel_val, return_state_vector=True, maximum_seconds=run_time, fps=fps,
                          headless=True, starting_q1=q1_val, starting_q2=q2_val, gravity=gravity,
                          q1_friction=q1_friction_, q2_friction=q2_friction_)


def drange(start, stop, step):
    """ Works just like xrange(), but accepts floats. Stops once the incremented value is larger than the stop value"""
    r = start
    while r < stop:
        yield r
        r += step


def get_angles_no_torque(table, index):
    angle1 = map_to(index[0], 0, table.shape[0] - 1, q1_low, q1_high)
    angle2 = map_to(index[1], 0, table.shape[1] - 1, q2_low, q2_high)
    angle1_vel = map_to(index[2], 0, table.shape[2] - 1, q1_vel_low, q1_vel_high)
    angle2_vel = map_to(index[3], 0, table.shape[3] - 1, q1_vel_low, q1_vel_high)
    return list((angle1, angle2, angle1_vel, angle2_vel))


def map_to(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


if __name__ == '__main__':
    if len(sys.argv) == 0:
        main(None)
    else:
        for i in range(1, len(sys.argv)):
            main(int(sys.argv[i]))
