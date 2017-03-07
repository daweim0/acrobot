from serial import Serial
import time
import math
import sys
import os
import numpy as np
from struct import unpack
import struct
from acrobot_state import State
import array

q1_offset = 0.5

custom_led = False

__docformat__ = 'restructuredtext en'


def count2time(t):
    # """ Returns seconds from micro controller time . """
    return t / 16e6


def deg_to_rad(angle):
    return angle * (np.pi / 180)


def rad_to_deg(angle):
    return angle * (180 / np.pi)


def map_to(x, in_min, in_max, out_min, out_max):
    return (float(x) - in_min) * (float(out_max) - out_min) / (float(in_max) - in_min) + out_min


class acrobot:
    """
    Hardware interface for the Acrobot. Most methods are self explanatory (no turbo encabulators).
    """

    def __init__(self, dev: int = 0):
        """ connect to the robot
        :param dev: serial port number (when on linux)
        """
        self.dev = ""
        self.power = 0
        self.lo_period = 16357  # approximates, but are temperature
        self.up_period = 16557  # dependent, so call .calibrate()
        self.LEN = 8  # number of samples sent
        self.CLK = 16e6  # ATMEGA128 clock freq
        self.MAX_SUM_COUNT = 1.0  # number of samples sent in a sum
        self.SAMPLES = 16.0  # number of samples between sums
        self.bytes_read = 0  # debug counter

        self.GYRO_OFFSET = 28
        self.GYRO_SCALAR = 0.001070423
        self.ACCELERATION_SCALAR = 10

        self.baud_rate = 57600*2
        # self.baud_rate = 2000000
        self.return_num = 0

        if dev:
            self.dev = dev
        elif os.name == "posix":
            try:
                self.dev = "/dev/" + list(filter(lambda a: 'ttyUSB' in a, os.listdir('/dev')))[0]
                # self.dev = serialenum.enumerate()[0]
            except BaseException as e:
                print("No FTDI devices found")
                print(e)
                sys.exit(1)
        else:  # probably running on windows, should probably add a check for os2
            self.dev = "COM3"
        # """Start serial port"""
        # Try connecting to it
        try:
            self.s = Serial(self.dev, self.baud_rate, timeout=99999.0)
            self.s.flushInput()
            self.s.flushOutput()
        except:
            print("Couldn't open connection to " + str(self.dev))
            sys.exit(1)

        # Ping to make sure it is plugged in
        try:
            self.ping()
        except struct.error as error:
            print("ping failed. Is the acrobot plugged in?")
            print(error)

        # Calibrate for current temperature
        self.calibrate()

        self.last_servo_time = time.time()
        self.last_servo_vel = 0

        return

    def calibrate(self):
        """ Compensate for temperature differences that can cause the rotary encoders to drift
        (read how often the enocders send pulses to figure out how off their internal clock is)"""
        lo_period_raw = []
        up_period_raw = []
        for i in range(1):
            self.s.write(chr(127).encode())
            lo_period_raw += self.__get_reading()
            up_period_raw += self.__get_reading()

        if np.amax(lo_period_raw) > 17000 or np.amax(up_period_raw) > 17000 \
                or np.amin(lo_period_raw) < 15000 or np.amin(up_period_raw) < 15000:
            # raise ValueError('calibration data was faulty')
            pass
            #TODO: put exception back

        # only set self.lo_period and up_period if the sensors are plugged in
        if np.average(lo_period_raw) > 10:
            self.lo_period = np.average(lo_period_raw)

        if np.average(lo_period_raw) > 10:
            self.up_period = np.average(lo_period_raw)
        return self.lo_period, self.up_period

    def servo_loop(self, target_acceleration_raw):
        target_acceleration = target_acceleration_raw
        target_acceleration *= self.ACCELERATION_SCALAR
        target_acceleration = int(clamp(round(target_acceleration), -126, 126))
        if target_acceleration < 0:  # encode power "map(power, 0, 128, 128, 0)"
            target_acceleration = -target_acceleration + (1 << 7)

        self.s.write(pack_number(0x01))
        self.s.write(pack_number(0x0d))
        self.s.write(array.array('B', [target_acceleration]).tostring())
        new_state = self.__read_return_data__(power = target_acceleration)

        return new_state

    # """program ending cleanup"""
    def __del__(self):
        try:
            self.write_read(0)  # power off
        except Exception:
            pass
        try:
            self.s.close()
        except Exception:
            pass

    def __get_angle_low(self, high, total):
        """ Turns pwm periods into angles
        :param high: pwm period
        :param total: maximum period from calibration data
        :return: angle in radians
        :rtype: float
        """
        angle = int(1026.0 * high / total - 1)
        if angle > 1022:
            angle = 1023
        return map_to(angle, 111, 633, -1 * math.pi/2, math.pi/2)

    def __get_angle_high(self, high, total):
        """ Turns pwm periods into angles
        :param high: pwm period
        :param total: maximum period from calibration data
        :return: angle in radians
        :rtype: float
        """
        angle = 1026.0 * high / total - 1
        if angle > 1022:
            angle = 1023
        return map_to(angle, 183, 696, -1 * math.pi/2, math.pi/2)

    def __get_reading(self, n=None):
        if n is None:
            n = self.LEN
        raw = self.s.read(2*n)
        data = list(unpack(">" + "H"*n, raw))
        # print "bytes", self.bytes_read, "to", self.bytes_read + n*2, " = ", data
        self.bytes_read += n*2
        return data

    def __read_gyro(self, n=6):
        raw = self.s.read(2*n)
        data = list(unpack(">" + "h"*n, raw))
        # print "bytes", self.bytes_read, "to", self.bytes_read + n*2, " = ", data
        self.bytes_read += n*2
        return data

    def __find_slope(self, data, period):
        t = np.arange(self.LEN) * period * self.SAMPLES / self.CLK
        orrigional_data = data
        #
        # average = np.average(data)
        # stdev = np.std(data)
        # data_list = list()
        # t_list = list()
        # for i in range(len(data)):
        #     if abs(data[i] - average) <= stdev + 0.001:
        #         data_list.append(data[i])
        #         t_list.append(t[i])
        # data = data_list
        # t = t_list

        # Find a cubic fit to get a slope for the second to last data point.
        # a, b, c, i = np.polyfit(t, data, 3)
        # lt = t[-2]
        # v0 = 3 * a * t[-3] ** 2 + 2 * b * t[-3] + c
        # v1 = 3 * a * t[-2] ** 2 + 2 * b * t[-2] + c
        # v2 = 3 * a * t[-1] ** 2 + 2 * b * t[-1] + c
        # va = (v0 + v1 + v2)/3.0
        # Use mean of last three points for the position - this introduces a
        # small delay into the position data.
        av = np.average(data[-4:])
        # a, b, c = np.polyfit(t, data, 2)
        # vb = 2 * a * lt + b

        a, b = np.polyfit(t, data, 1)
        return a, b + a * t[-1]

    def write_read(self, power: float) -> State:
        """
        Sets the motor power to the passed integer and returns the two joint angles and velocities
        :param power: power to write to the motor (between -127 and 127, 0 is off)
        :return: [lower velocity, upper velocity, lower angle, upper angle]
        :rtype: [float, float, float, float]
        """
        power_raw = power
        if power is not None:
            power = int(round(power))
            if power < 0:  # encode power "map(power, 0, 128, 128, 0)"
                power = -power + (1 << 7)
            if power == 127:  # inhibit calibration
                power = 126
            if power == 1:  # inhibit advanced commands
                power = 2
            self.power = power
        self.s.flushInput()

        start_time = time.clock()

        self.s.write(array.array('B', [self.power]).tostring())

        return self.__read_return_data__(power = power_raw)

    def __read_return_data__(self, power = 0):
        # self.lo and self.up will contain the last LEN data points, oldest in
        # index 0.
        raw_lo = np.array(self.__get_reading()) / self.MAX_SUM_COUNT
        raw_up = np.array(self.__get_reading()) / self.MAX_SUM_COUNT
        # print_tuple(self.up, precision=1, new_line=True)

        gyrodata = self.__read_gyro()

        end_time = time.clock()

        # print(np.mean(raw_up), self.up_period, '  ', raw_up)

        processed_lo = np.ones(raw_lo.shape)
        processed_up = np.ones(raw_up.shape)

        # Convert to angles
        for i in range(len(processed_lo)):
            processed_lo[i] = self.__get_angle_low(raw_lo[i], self.lo_period)
            processed_up[i] = self.__get_angle_high(raw_up[i], self.up_period)

        self.lo = processed_lo
        self.up = processed_up

        v_lo, lo = self.__find_slope(processed_lo, self.lo_period)
        v_up, up = self.__find_slope(processed_up, self.up_period)

        # v_up = (gyrodata[0] - self.GYRO_OFFSfET) * (15.2 / 14200)

        v_lo, lo = 0, math.pi/2


        self.return_num += 1
        # self.last_state = State([lo, up, v_lo, v_up * 16.5, power_raw], id_num=self.return_num, time=time.time(), gyrodata=gyrodata)
        self.last_state = State([lo, up, v_lo, (gyrodata[0] + self.GYRO_OFFSET) * self.GYRO_SCALAR, power], id_num=self.return_num, time=time.time(), gyrodata=gyrodata)


        return self.last_state

    def read_last_data(self):
        return [self.last_lo, self.last_up, self.last_v_lo, self.last_v_up]

    # process data from sources other than the hardware
    def inject(self, los, ups):
        """
            Insert fake data (from sources other than the hardware)
        """
        self.lo = los
        self.up = ups
        v_lo, lo = self.find_fit(los, self.lo_period)
        v_up, up = self.find_fit(ups, self.up_period)
        return [lo, up, v_lo, v_up]

    def get_vals(self):
        """ Gets updated angle measurements without writing a new power"""
        return self.write_read(self.power)

    # """TODO: Check for filter_send()'s existence"""
    def get_filter_vals(self):
        return self.filter_send(self.power)

    def stop(self):
        """ turns off power to the motor """
        return self.write_read(0)

    def __str__(self):
        vals = self.get_vals()
        return "U: %3.4f [d %3.4f], L: %3.4f [d %3.4f], P: %d" % (vals[3], vals[1], vals[2], vals[0], self.power)

    def set_led(self, input) -> None:
        """
        Sets the color on the status led on the acrobot. The strings "red", "green",
        "blue", or an integer can be passed. If an integer is passed then the lsb turns blue on,
         the second bit turns green led, and the third bit turns red on.
        :param input: can be a string or integer.
        """
        self.s.write(pack_number(0x01))  # general command prefix
        self.s.write(pack_number(11))  # set led prefix
        if input == "red":
            self.s.write(pack_number(0x04))
        elif input == "green":
            self.s.write(pack_number(0x02))
        elif input == "blue":
            self.s.write(pack_number(0x01))
        elif ord(chr(input)) == input and 0 <= input <= 7:  # check to see if input is a byte
            self.s.write(pack_number(input))
        else:
            raise ValueError("malformed input passed to set_led()")

    def ping(self) -> bytes:
        """
        Sends a ping packet to the acrobot and returns it's response.
        :return: Byte that the acrobot responds to the ping with
        """
        self.s.write(pack_number(0x01))
        self.s.write(pack_number(12))
        return self.s.read(1)

    def exit(self):
        self.stop()
        self.s.close()


def pack_number(number):
    """ Formats a number into a string for printing to a serial connection """
    return array.array('B', [number]).tostring()


def remove_outliers(input_list: (), stdev_bound: float = 1.5) -> ():
    """
    Removes statistical outliers from the passed list. The passed list will not be modified. Element order is preserved.
    :param input_list: list to remove outliers from
    :param stdev_bound: number of standard deivations away from the mean a value must be in order to be removed.
    """
    temp2 = list()
    stdev = np.std(input_list)
    average = np.mean(input_list)
    count = 0
    for i in range(len(input_list)):
        if abs(input_list[i] - average) <= stdev * stdev_bound:
            count += 1
    output = np.zeros([count])
    elements_filled = 0
    for i in range(len(input_list)):
        if abs(i) <= stdev * stdev_bound:
            output[elements_filled] = input_list(elements_filled)
            elements_filled += 1
    return output


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
                if i < 0:
                    precision += 1
    else:
        print((str(tup) + str(precision) + 'f'), end=' ')


def clamp(n, min_n, max_n):
    return max(min(max_n, n), min_n)