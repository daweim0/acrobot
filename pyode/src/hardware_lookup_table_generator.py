import warnings

__author__ = 'David'

__docformat__ = 'restructuredtext en'

import random
import os
import math
from math import pi
import time
import threading
import numpy
import queue
import hardware
from subprocess import call
import operator
import simulation
import lookup_table_hopper as hopper
from acrobot_state import State
import threading
from os import listdir
import ast
import re
import pickle

table_number = 301


# declare global variables here for
# q1_low, q1_high, q2_low, q2_high, q1_vel_low, q1_vel_high, q2_vel_low, q2_vel_high, torque_low, torque_high = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
# table_dim_low, table_dim_high, resolution, velocity_resolution, torque_resolution, fps, dt, run_time = [], [], 0, 0, 0, 0, 0, 0


class LookupTableFiller:
    def __init__(self, logging=False, use_simulation=True):

        # global q1_low, q1_high, q2_low, q2_high, q1_vel_low, q1_vel_high, q2_vel_low, q2_vel_high, torque_low, torque_high
        # global table_dim_low, table_dim_high, resolution, velocity_resolution, torque_resolution, fps, dt, run_time

        self.use_simulation = use_simulation
        self.data_input_queue = queue.Queue(maxsize=1000)  # for incoming data (possibly from other threads)
        self.data_queue = list()  # incoming data is transered here from self.data_input_queue

        self.torque_dbg_queue = list()

        if self.use_simulation:
            self.q1_low = 0
            self.q1_high = math.pi * 2
            self.q2_low = -1 * math.pi
            self.q2_high = math.pi
            self.q1_vel_low = -2.0
            self.q1_vel_high = 2.0
            self.q2_vel_low = -3.0
            self.q2_vel_high = 3.0
            self.torque_low = -1.0
            self.torque_high = 1.0

            self.resolution = 0.2  # resolution of the lookup table in radians
            self.velocity_resolution = 0.2
            self.torque_resolution = 0.5  # resolution to step torques by
            self.fps = 10
            self.dt = 1.0 / self.fps
        else:
            self.q1_low = 0 * pi / 180
            self.q1_high = 180 * pi / 180
            self.q2_low = -70 * pi / 180
            self.q2_high = 70 * pi / 180
            self.q1_vel_low = -0.3
            self.q1_vel_high = 0.3
            self.q2_vel_low = -1.0
            self.q2_vel_high = 1.0
            self.torque_low = -40
            self.torque_high = 40

            self.resolution = 0.15  # resolution of the lookup table in radians
            self.velocity_resolution = 0.15
            self.torque_resolution = 10  # resolution to step torques by
            self.fps = 15
            self.dt = 1.0 / self.fps

        self.table_dim_low = (self.q1_low, self.q2_low, self.q1_vel_low, self.q2_vel_low, self.torque_low)
        self.table_dim_high = (self.q1_high, self.q2_high, self.q1_vel_high, self.q2_vel_high, self.torque_high)

        self.error_margin = 0.1  # 10%
        if logging:
            self.log_file = open('./log_recorder.txt', 'a')
        self.lock = threading.Lock()

        self.table_history = list()

        try:
            if os.name == 'posix':
                if self.use_simulation:
                    # self.table = numpy.load(
                    #     "table" + str(table_number) + "_[1.45, 1.65, -0.2, 0.2, -0.8, 0.8, -1.25, 1.25, "
                    #                                   "-1, 1, 0.01, 0.025, 0.25, 10, 0, 0].npy")
                    self.table = numpy.load(get_table_name(table_number))
                else:
                    self.table = numpy.load(
                        "table" + str(table_number) + "_[0.8726646259971648, 2.0 2.0, -4.0, 4.0, "
                                                      "-80, 80, 0.15, 0.25, 20, 10, 0, 0].npy")
            else:
                if self.use_simulation:
                    # self.table = numpy.load(
                        # "table" + str(table_number) + "_[0.8726646259971648, 2.0943951023931953, -0.8726646259971648,"
                        #                               " 0.8726646259971648, -2.0, 2.0, -4.0, 4.0, -100, 100, 0.1, "
                        #                               "0.2, 10, 10, 0, 0].npy")
                    self.table = numpy.load(get_table_name(table_number))
                else:
                    self.table = numpy.load(
                        "table" + str(table_number) + "_[1.0471975511965976, 1.7453292519943295, -1.2217304763960306,"
                                                      " 1.2217304763960306, -0.3, 0.3, -1.0, 1.0, "
                                                      "-40, 40, 0.15, 0.15, 10, 15, 0, 0].npy")

        except:
            self.table = numpy.zeros(
                (int(math.floor(self.q1_high / self.resolution)) - int(math.floor(self.q1_low / self.resolution)),
                 int(math.floor(self.q2_high / self.resolution)) - int(math.floor(self.q2_low / self.resolution)),
                 int(math.floor(self.q1_vel_high / self.velocity_resolution)) - int(
                     math.floor(self.q1_vel_low / self.velocity_resolution)),
                 int(math.floor(self.q2_vel_high / self.velocity_resolution)) - int(
                     math.floor(self.q2_vel_low / self.velocity_resolution)),
                 int(math.floor(self.torque_high / self.torque_resolution)) - int(
                     math.floor(self.torque_low / self.torque_resolution)),
                 4), dtype=numpy.float32)  # 4 values in state vector

        hopper.initialize(None, imported_table=self.table)
        hopper.set_parameters(q1_low_=self.q1_low,
                              q1_high_=self.q1_high,
                              q2_low_=self.q2_low,
                              q2_high_=self.q2_high,
                              q1_vel_low_=self.q1_vel_low,
                              q1_vel_high_=self.q1_vel_high,
                              q2_vel_low_=self.q2_vel_low,
                              q2_vel_high_=self.q2_vel_high,
                              torque_low_=self.torque_low,
                              torque_high_=self.torque_high,
                              resolution_=self.resolution,
                              velocity_resolution_=self.velocity_resolution,
                              torque_resolution_=self.torque_resolution)

        self.average_n = 0
        self.accuracy_average = 0.0
        self.autofiller = None  # signifies that the autofiller isn't running
        self.sugestion_set = set()

    def close(self):
        try:
            print("accuracy average =", self.accuracy_average / self.average_n, "with ", self.average_n, "measurements")
        except ZeroDivisionError:
            pass
        print("table entries =", len(self.table[self.table != 0]) / 4)
        numpy.save('table' + str(table_number) + '_' + str(
            [self.q1_low, self.q1_high, self.q2_low, self.q2_high, self.q1_vel_low, self.q1_vel_high,
             self.q2_vel_low, self.q2_vel_high, self.torque_low, self.torque_high,
             self.resolution, self.velocity_resolution, self.torque_resolution,
             self.fps, 0, 0]), self.table)
        print("done saving table " + str(table_number) + ", exiting with", len(self.table[self.table != 0]) / 4,
              "entries")

    def feed_data(self, new_state: State):
        self.data_input_queue.put(new_state)
        if self.autofiller is None:
            self.iterate()

        # angles = new_state.angles
        # torque = new_state.angles[4]
        # state_time = new_state.time
        # id_num = new_state.id_num
        # self.data_old = angles
        # self.n_ticks += 1
        # self.auto_filler.feed_data(angles, torque, state_time, id_num)

    def __del__(self):
        pass

    def iterate(self):
        # self.lock.aquire()
        while not self.data_input_queue.empty():
            self.data_queue.append(self.data_input_queue.get())

        # make sure we waited long enough between measurements
        while len(self.data_queue) > 1:

            self.__iterate_helper__(self.data_queue[0], self.data_queue[1])

            # self.log_file.write(str(self.data_queue[0]) + "\n")
            self.data_queue.pop(0)
        # self.lock.release()
        # print("lookup table entries: " + str(len(numpy.argwhere(self.table)) / 4))

    def __iterate_helper__(self, start_state, end_state, dry_run=False):
        if abs(end_state.time - start_state.time) < self.dt * (1 + self.error_margin) or True:

            # Make a better approximation of the acrobot's position at exactly dt seconds after data_queue[0] was
            # recorded. This is done by making a linear approximation for position vs time then calculating the
            # position at data_queue[0].time + dt
            starting_position = start_state.angles[0:4]
            torque = start_state.angles[4]
            dt = end_state.time - start_state.time
            dx = (numpy.array(end_state.angles[0:4]) - numpy.array(starting_position)) / dt
            ending_position = dx * self.dt + numpy.array(starting_position)
            ending_index = hopper.get_index_interpolated(ending_position)

            index = hopper.get_index_interpolated(list(start_state.angles))
            for i in range(len(index)):
                index[i] = int(round(index[i], 1))

            if hopper.is_within_bounds(index):

                self.sugestion_set |= {str((*index[0:4], index[4]))}

                if self.table[tuple(index)][0] == 0.0:
                    print("wrote", index, "->", ending_index, "first time")
                    if not dry_run:
                        self.table[tuple(index)] = ending_position

                    for i in range(len(self.torque_dbg_queue)):
                        if self.torque_dbg_queue[i][1] == index[4]:
                            self.torque_dbg_queue.pop(i)
                            break
                    # print "\tset", index, "to", self.table[index[0], index[1], index[2], index[3], index[4]]
                    # print "**************************************************************"
                    # print "new data added to lookup table at", index
                    # print "new data = ", data
                    # print "**************************************************************"
                    # self.log_file.write("(new), id, " + str(old_data[3]) + ", " + str(new_data[3]) + ", dt, " +
                    #                     str(new_data[2] - old_data[2]) + ", " + str(old_angles) + ", " + str(
                    #     old_torque) +
                    #                     " (index " + str(index) + ") to " + str(data) + "\n")
                    self.table_history.append({'starting_state': starting_position, 'ending_state': ending_position,
                                               'starting_index': hopper.get_index_interpolated(list(start_state.angles)),
                                               'starting_index_rounded': index, 'ending_index': ending_index,
                                               'torque': torque, 'dx': dx, 'dt': dt})

                # foo = open('dump1.txt', 'w')
                # for i in self.table_history:
                #     foo.write(str(i) + ' \n')

                else:
                    # print("wrote", index, "->", ending_index)
                    # current_value = self.table[tuple(index)]
                    # prediction_error = hopper.dist_between(ending_position, list(current_value), weight_set=1)
                    # self.average_n += 1
                    # self.accuracy_average += prediction_error
                    #
                    # for i in range(len(self.torque_dbg_queue)):
                    #     if self.torque_dbg_queue[i][1] is None:
                    #         self.torque_dbg_queue.pop(i)
                    #         break
                    # # print "**************************************************************"
                    # # print self.accuracy_average / self.average_n
                    # # print prediction_error
                    # # lookup_table_hopper.print_tuple(current_value - numpy.array(data), precision=5, new_line=True)
                    # # print "**************************************************************"
                    # # continuously running weighted average
                    # if not dry_run:
                    #     self.table[tuple(index)] = current_value * .9 + ending_position * .1
                    pass
            else:
                if not dry_run:
                    print("wrote nothing for", index, " (out of bounds)")

        if dry_run:
            return index

    def get_torque(self, postition, dry_run=False):
        """
        Returns a torque that would fill new data in the lookup table
        :param postition: [float, float, float, float]
        """
        index = list(hopper.get_index_interpolated(postition))
        for i in range(len(index)):
            index[i] = int(round(index[i], 1))
        return_torque = None
        current_torque = None
        if hopper.is_within_bounds(index):
            possible_torques = list(range(self.table.shape[4]))

            while len(possible_torques) != 0:
                current_torque = possible_torques.pop(int(len(possible_torques) / 2))
                if -0.000001 < self.table[index[0], index[1], index[2], index[3], current_torque, 0] < 0.000001:
                    return_torque = hopper.get_torque_val(current_torque)
                    break

        if not dry_run:
            self.torque_dbg_queue.append((current_torque, return_torque))
            if return_torque is not None and str((*index, return_torque)) in self.sugestion_set:
                print("VALUE ADDED TO TABLE RETURNED AS SUGESTION", str((*index, return_torque)))
            print("sugested torque index", current_torque, "for index", index, " (numerically", return_torque, ")")
            return return_torque
        else:
            return return_torque, index

    def test_get_torque_and_iterate_helper(self):
        for _ in range(1000):
            starting_index = State(angles=list((map_to(random.random(), 0, 1, self.q1_low, self.q1_high),
                          map_to(random.random(), 0, 1, self.q2_low, self.q2_high),
                          map_to(random.random(), 0, 1, self.q1_vel_low, self.q1_vel_high),
                          map_to(random.random(), 0, 1, self.q2_vel_low, self.q2_vel_high))))
            ideal_torque, get_torque_index = self.get_torque(starting_index, dry_run=True)
            if ideal_torque is not None:
                starting_index.angles.append(ideal_torque)
                iterate_index = self.__iterate_helper__(State(angles=starting_index), State(angles=starting_index), dry_run=True)
                if not (iterate_index[0:4] == get_torque_index[0:4]) or not (self.table[tuple(iterate_index)][0] == 0.0):
                    self.get_torque(starting_index, dry_run=True)
            else:
                starting_index.angles.append(0)
                iterate_index = self.__iterate_helper__(State(angles=starting_index), State(angles=starting_index), dry_run=True)
                assert(iterate_index[0:4] == get_torque_index[0:4])
                assert(self.table[tuple(iterate_index[0:4])][0][0] == 0.0)

    # this class should loop checking to see if there is enough data to fill in the lookup table
    # TODO: update this code
    class AutoFiller(threading.Thread):

        def __init__(self):
            self.last_data = None
            self.fps = 10
            self.running = True
            self.robot = None
            self.done_queue = queue.Queue(maxsize=1)
            threading.Thread.__init__(self)

        def feed_data(self, state, torque, time, id_num):
            if len(state) == 5:
                torque = state[4]
            if self.data_queue.qsize() == self.data_queue.maxsize:
                raise Exception("AutoFiller input queue filled up")
            self.data_queue.put([state[0:4], torque, time, id_num])

        def stop(self):
            self.running = False

        def run(self):
            """
            Inserts new data into the data table
            """
            while self.running:
                self.iterate()
                time.sleep(0.005)

            self.log_file.close()


def map_to(x, in_min, in_max, out_min, out_max):
    return (float(x) - in_min) * (float(out_max) - out_min) / (float(in_max) - in_min) + out_min


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
    return name



    # def get_index_interpolated(self, table, angles):
    #     """
    #     Converts angles to lookup table indicies.
    #     :param angles: [float, float, float, float]
    #     :return: [float, float, float, float]
    #     """
    #     angle1 = map_to(angles[0], q1_low, q1_high, 0.0, table.shape[0] - 1)
    #     angle2 = map_to(angles[1], q2_low, q2_high, 0.0, table.shape[1] - 1)
    #     angle1_vel = map_to(angles[2], q1_vel_low, q1_vel_high, 0.0, table.shape[2] - 1)
    #     angle2_vel = map_to(angles[3], q2_vel_low, q2_vel_high, 0.0, table.shape[3] - 1)
    #     if len(angles) == 5:
    #         return list(
    #             (angle1, angle2, angle1_vel, angle2_vel,
    #              map_to(angles[4], torque_low, torque_high, 0, table.shape[4] - 1)))
    #     return int(round(angle1)), int(round(angle2)), int(round(angle1_vel)), int(round(angle2_vel))


    # def get_angles(table, index):
    #     """
    #     :param index: acrobot table indicies
    #     :return: acrobot angles (4 element tuple)
    #     """
    #     angle1 = map_to(index[0], 0.0, table.shape[0] - 1, q1_low, q1_high)
    #     angle2 = map_to(index[1], 0.0, table.shape[1] - 1, q2_low, q2_high)
    #     angle1_vel = map_to(index[2], 0.0, table.shape[2] - 1, q1_vel_low, q1_vel_high)
    #     angle2_vel = map_to(index[3], 0.0, table.shape[3] - 1, q2_vel_low, q2_vel_high)
    #     if len(index) == 5:
    #         return list((angle1, angle2, angle1_vel, angle2_vel,
    #                      map_to(index[4], 0, table.shape[4] - 1, torque_low, torque_high)))
    #     return list((angle1, angle2, angle1_vel, angle2_vel))
    #
    #
    # def get_torque_val(table, torque_index):
    #     """
    #     Converts a torque table index to a value.
    #     :rtype: float
    #     :type torque_index: float
    #     """
    #     return map_to(torque_index, 0, table.shape[4] - 1, torque_low, torque_high)
    #
    #
    # def get_torque_index(table, torque_val):
    #     """ :rtype: float """
    #     return map_to(torque_val, torque_low, torque_high, 0, table.shape[4] - 1)


    # def main():
    #     setup()
    #     data_store.accuracy_average = 0
    #     data_store.average_n = 0
    #
    #     if use_simulation:
    #         robot = SimulationThread(run_time=9999, fps=fps).start()
    #         time.sleep(1.0 / fps * 2)
    #         while len(simulation.position_output_queue) == 0:
    #             time.sleep(0.001)
    #         data = simulation.position_output_queue.pop(0)
    #         data_store.data_old = [data[0], data[1], data[2], data[3]]
    #     else:
    #         robot = hardware.acrobot()
    #         robot.stop()
    #         time.sleep(0.01)
    #         data = robot.write_read(0)
    #         data_store.data_old = [data[0], data[1], data[2], data[3]]
    #         time.sleep(1.0 / fps)
    #     # time.sleep(10)
    #
    #     data_store.torque_old = 0
    #
    #     last_time = time.time()
    #
    #     for n_ticks in range(run_time * fps):
    #
    #         while last_time + 1.0 / fps > time.time():
    #             time.sleep(0.001)
    #
    #         if use_simulation:
    #             while len(simulation.position_output_queue) < 1:
    #                 time.sleep(0.01)
    #             data = simulation.position_output_queue.pop(0)
    #         else:
    #             data = robot.write_read(data_store.torque_old)
    #
    #         last_time = time.time()
    #         print("\t\t\t\t\treceived", data, "at time", float(n_ticks) / fps, "seconds")
    #
    #         index = list(get_index_interpolated(data_store.table, data_store.data_old))
    #         index.append(int(round(
    #             (map_to(data_store.torque_old, table_dim_low[4], table_dim_high[4], 0, data_store.table.shape[4] - 1)))))
    #         iterate(data)
    #
    #         if use_simulation:
    #             simulation.set_constant_torque(data_store.torque_old)
    #         if n_ticks % 50 == 0:
    #             print("\n\n\n\ntable entries =", len(data_store.table[data_store.table != 0]) / 4, "\n\n\n\n")
    #     if not use_simulation:
    #         robot.stop()
    #     else:
    #         simulation.loop_flag = False
    #     close()

# class SimulationThread(threading.Thread):
#     """ simulation.py was designed to be in charge, not called by other modules, so this thread is needed to manage it
#     (just an easy way to invert control) """
#
#     def __init__(self, run_time=99999, fps=15):
#         self.run_time = run_time
#         self.fps = fps
#         threading.Thread.__init__(self)  # run thread's constructor
#
#     def run(self):
#         simulation.run(starting_q1=270 * math.pi / 180,
#                        starting_q2=0.0 * math.pi / 180,
#                        starting_q1v=0.0,
#                        starting_q2v=0.0,
#                        q1_friction=0,
#                        q2_friction=1,
#                        headless=False,
#                        fps=self.fps,
#                        maximum_seconds=self.run_time,
#                        passive=True,
#                        constant_torque=0,
#                        has_bounds=False,
#                        exit_on_fail=False,
#                        noise=0.00,
#                        interactive=True,
#                        keep_time=True,
#                        gravity=-9.81)



# class DataStorageContainer:
#     def __init__(self):
#         self.data_old = None
#         self.torque_old = None
#         self.average_n = 0
#         self.accuracy_average = 0
#         self.n_ticks = 0
#         self.table = None
#         self.auto_filler = None
#         self.robot_interface = None


# data_store = DataStorageContainer()


# if __name__ == "__main__":
#     for i in range(3):
#         main()
#         # time.sleep(2.0)
#         # call(["python", "lookup_table_checker.py", "save"])
#         # time.sleep(2.0)
