import pickle
import numpy as np
import math
from math import pi


def main():
    table_history = pickle.load(open('data_dump.p', 'rb'))
    table = create_table()

    for point in table_history:
        table[tuple(point['starting_index_rounded'])] = (point['starting_state'], point['dx'])
    pass


def create_table() -> object:
    q1_low = 0 * pi / 180
    q1_high = 180 * pi / 180
    q2_low = -70 * pi / 180
    q2_high = 70 * pi / 180
    q1_vel_low = -0.3
    q1_vel_high = 0.3
    q2_vel_low = -1.0
    q2_vel_high = 1.0
    torque_low = -40
    torque_high = 40

    resolution = 0.15  # resolution of the lookup table in radians
    velocity_resolution = 0.15
    torque_resolution = 10  # resolution to step torques by
    fps = 15
    dt = 1.0 / fps

    table_dim_low = (q1_low, q2_low, q1_vel_low, q2_vel_low, torque_low)
    table_dim_high = (q1_high, q2_high, q1_vel_high, q2_vel_high, torque_high)

    raw_table =  np.ndarray((int(math.floor(q1_high / resolution)) - int(math.floor(q1_low / resolution)),
     int(math.floor(q2_high / resolution)) - int(math.floor(q2_low / resolution)),
     int(math.floor(q1_vel_high / velocity_resolution)) - int(
         math.floor(q1_vel_low / velocity_resolution)),
     int(math.floor(q2_vel_high / velocity_resolution)) - int(
         math.floor(q2_vel_low / velocity_resolution)),
     int(math.floor(torque_high / torque_resolution)) - int(
         math.floor(torque_low / torque_resolution))), dtype=np.object)  # 4 values in state vector

    def helper(table):
        if len(table.shape) > 1:
            for i in range(len(table)):
                helper(table[i])
            return
        else:
            for i in range(len(table)):
                table[i] = list()
            return

    helper(raw_table)

    return raw_table




if __name__ == '__main__':
    main()