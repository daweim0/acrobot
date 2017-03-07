import math
import random
import threading
import numpy
import ode
import queue
import time
from acrobot_state import State


q1_offset = 0.5
custom_led = False

__docformat__ = 'restructuredtext en'


class Cord(object):
    def __init__(self, x, y):
        self._x = x
        self._y = y

    def x(self):
        return self._x

    def y(self):
        return self._y

    def get(self):
        return self._x, self._y


l1 = 0.75
l2 = 1.0
lower_mass = 1.0
upper_mass = 1.0


class acrobot:
    """
    Hardware interface for the Acrobot. Most methods are self explanatory (no turbo encabulators).
    """

    def __init__(self, starting_q1=math.pi/2, starting_q2=0, starting_q1v=0.0, starting_q2v=0, gravity_tuple=[0, -9.8, 0], q1_friction=0, q2_friction=0):

        global body1, body2

        starting_y1 = math.sin(float(starting_q1)) * l1
        starting_x1 = math.cos(float(starting_q1)) * l1
        starting_x2 = starting_x1 + math.cos((float(starting_q1) + float(starting_q2))) * l2
        starting_y2 = starting_y1 + math.sin((float(starting_q1) + float(starting_q2))) * l2

        origin = Cord(0, 0)
        lower = Cord(origin.x() + starting_x1, origin.y() + starting_y1)
        upper = Cord(origin.x() + starting_x2, origin.y() + starting_y2)
        self.world = ode.World()
        self.world.setGravity(gravity_tuple)
        self.world.setERP(0.8)  # error correction, .8 is the maximum recommended value
        self.world.setCFM(0.000000001)  # makes joints slightly springy, helps in setting initial velocities

        # Create two bodies
        body1 = ode.Body(self.world)
        m1 = ode.Mass()
        m1.setSphereTotal(lower_mass, 1.0)
        body1.setMass(m1)
        body1.setPosition((lower.x(), lower.y(), 0))
        self.body1 = body1

        body2 = ode.Body(self.world)
        m2 = ode.Mass()
        m2.setSphereTotal(upper_mass, 0.01)
        body2.setMass(m2)
        body2.setPosition((upper.x(), upper.y(), 0))
        self.body2 = body2

        # Connect body1 with the static environment
        j1 = ode.HingeJoint(self.world)
        j1.attach(body1, ode.environment)
        j1.setAnchor((origin.x(), origin.y(), 0))
        j1.setAxis((0, 0, 1))

        # Connect body2 with body1
        j2 = ode.HingeJoint(self.world)
        j2.attach(body1, body2)
        j2.setAnchor((lower.x(), lower.y(), 0))
        j2.setAxis((0, 0, 1))

        torque_dt = 0.00001
        accuracy = 0.00025
        for step_num in range(0, 15):
            q2 = -1 * (j2.getAngle() + -1 * float(starting_q2))
            q1 = j1.getAngle() + float(starting_q1)
            q1_vel = body1.getLinearVel()
            q2_vel = body2.getLinearVel()

            w1 = starting_q1v
            w2 = starting_q2v

            v1 = w1 * l1
            v2 = w2 * l2

            v1x = math.sin(q1) * v1
            v1y = math.cos(q1) * v1

            p1 = numpy.array((math.cos(q1) * l1, math.sin(q1) * l1))
            p2_rel = numpy.array((math.cos(q1 + q2) * l2, math.sin(q1 + q2) * l2))
            p2 = p1 + p2_rel
            d = math.sqrt(p2[0] ** 2 + p2[1] ** 2)
            theta_d = math.acos(p2[0] / d)
            if p2[1] < 0:
                theta_d = math.pi * 2 - theta_d

            v2x = v2 * math.sin(q1 + q2) + d * w1 * math.sin(theta_d)
            v2y = v2 * math.cos(q1 + q2) + d * w1 * math.cos(theta_d)

            body1.setLinearVel((-1 * v1x, v1y, 0))
            body2.setLinearVel((-1 * v2x, v2y, 0))

            # For debugging
            # q1_diff = math.atan(v1x/v1y) - q1
            # q2_diff = math.atan((v2x-v1x)/(v2y-v1y)) - (q1 + q2)
            self.world.step(torque_dt)

            if ((abs(q1_vel[0] - v1x) < accuracy and abs(q1_vel[1] - v1y) < accuracy) and (
                            abs(q2_vel[0] - v2x) < + accuracy and abs(q2_vel[1] - v2y) < accuracy)):
                break

        self.world.setCFM(0.0000001)  # Turn joint springiness back down

        self.output_queue = queue.Queue(maxsize=100)
        self.simulation = SimulationThread(output_queue=self.output_queue, world=self.world, j1=j1, j2=j2,
                                           starting_q1=starting_q1, starting_q2=starting_q2, m1=body1, m2=body2,
                                           q1_friction=q1_friction, q2_friction=q2_friction)
        self.torque_queue = self.simulation.get_torque_queue()
        self.simulation.start()

        self.last_data = [0.0, 0.0, 0.0, 0.0]

    def calibrate(self):
        pass

    def __del__(self):
        self.simulation.close()

    def __get_angle(self, high, total):
        pass

    def __get_reading(self, n=None):
        pass

    def __find_slope(self, data, period):
        pass

    def write_read(self, power):
        """
        Sends the motor power to the passed integer and returns the two joint angles and velocities
        :param power: power to write to the motor (between -127 and 127, 0 is off)
        :return: [lower velocity, upper velocity, lower angle, upper angle]
        :rtype: [float, float, float, float]
        """
        if power is not None:
            self.torque_queue.put(power)
        # self.read_last_data()
        # return self.simulation.output_vals
        self.simulation.output_vals[4] = 0
        return State(self.simulation.output_vals[0:5], time=time.time(), gyrodata=[0, 0, 0])

    def request_data(self, power):
        pass

    def read_last_data(self):
        try:
            self.last_data = self.output_queue.get()
        except queue.Empty:
            pass

    # process data from sources other than the hardware
    def inject(self, los, ups):
        pass

    def get_vals(self):
        pass

    def get_filter_vals(self):
        pass

    def stop(self):
        """ turns off power to the motor """
        for _ in range(100):
            if self.torque_queue.empty():
                break
            self.torque_queue.get(False)
        self.torque_queue.put(0)

    def set_led(self, input):
        pass

    def exit(self):
        self.stop()
        self.simulation.close()

    def ping(self):
        pass


class SimulationThread(threading.Thread):
    """ Just exists because simulation was designed to be in charge, not called by other modules
    (just an easy way to invert control) """

    def __init__(self, output_queue=None, world=None, j1=None, j2=None, run_time=9999999, fps=30, q1_friction=0.0,
                 q2_friction=0.0, noise=0.0, starting_q1=0.0, starting_q2=0.0, m1=None, m2=None):
        self.run_time = run_time
        self.fps = fps
        self.output_queue = output_queue
        self.world = world
        self.j1 = j1
        self.j2 = j2
        self.starting_q1 = starting_q1
        self.starting_q2 = starting_q2
        self.noise = noise
        self.q1_friction = q1_friction
        self.q2_friction = q2_friction
        self.m1 = m1
        self.m2 = m2
        self.torque_queue = queue.Queue(maxsize=100)
        self.current_torque = 0.0
        self.time_step = 1.0 / self.fps
        self.closed = False
        self.log_file = open('./log_ode.txt', 'a')
        self.step_num = 0

        self.last_position = list((math.pi / 2, 0, 0, 0))

        threading.Thread.__init__(self)  # run thread's constructor

    def get_torque_queue(self):
        return self.torque_queue

    def run(self):
        # Next simulation step
        n_substeps = 10

        last_time = time.time()
        while not self.closed:
            try:
                self.current_torque = self.torque_queue.get_nowait()
            except queue.Empty:
                pass

            dt = time.time() - last_time
            last_time = time.time()
            # dt = 1.0/self.fps

            # self.last_position[0] += self.last_position[1] * dt
            # self.last_position[0] = constrain(self.last_position[0], 70 * math.pi / 180, 105 * math.pi / 180)
            # self.last_position[0] = (self.last_position[0] - math.pi/2) * (1 - 0.75 * dt) + math.pi/2
            #
            # self.last_position[1] += self.current_torque * dt
            # self.last_position[1] = constrain(self.last_position[1], -30 * math.pi / 180, 30 * math.pi / 180)
            # self.last_position[1] *= (1 - 0.75 * dt)
            #
            # self.last_position[2] += self.last_position[3] * dt
            # self.last_position[2] = constrain(self.last_position[2], -2.0, 2.0)
            # self.last_position[2] *= (1 - 0.75 * dt)
            #
            # self.last_position[3] += self.current_torque * dt * 10
            # self.last_position[3] = constrain(self.last_position[3], -4.0, 4.0)
            # self.last_position[3] *= (1 - 0.75 * dt)

            starting_state = [self.j1.getAngle() + (random.random() - .5) * self.noise + float(self.starting_q1),
                                   -1 * (self.j2.getAngle() + (random.random() - .5) * self.noise + -1 * float(
                                   self.starting_q2)), self.j1.getAngleRate() + (random.random() - .5) * self.noise,
                                   -1 * self.j2.getAngleRate() + (random.random() - .5) * self.noise]
            applied_torque = self.current_torque
            for _ in range(n_substeps):
                self.j2.addTorque(-1 * applied_torque)
                # bearing friction
                self.j2.addTorque(-1 * self.j2.getAngleRate() * self.q2_friction)
                self.j1.addTorque(-1 * self.j1.getAngleRate() * self.q1_friction)
                # Keeps the lower segment upright for testing
                self.j1.addTorque(-1 * (self.j1.getAngle() + float(self.starting_q1) - math.pi/2) * 20)
                self.world.step(self.time_step / n_substeps)

                # virtual safety net
                # anchor = [0, 1.75]
                # m2_position = self.m2.getPosition()
                # m2_vel = self.m2.getLinearVel()
                # m2dist = math.sqrt(math.pow(m2_position[0] - anchor[0], 2) + math.pow(m2_position[1] - anchor[1], 2))
                # if m2dist > 1:
                #     v = math.sqrt(math.pow(m2_vel[0], 2) + math.pow(m2_vel[1], 2))

            if self.output_queue.full():
                self.output_queue.get()
            new_state = list([self.j1.getAngle() + (random.random() - .5) * self.noise + float(self.starting_q1),
                                   -1 * (self.j2.getAngle() + (random.random() - .5) * self.noise + -1 * float(
                                   self.starting_q2)), self.j1.getAngleRate() + (random.random() - .5) * self.noise,
                                   -1 * self.j2.getAngleRate() + (random.random() - .5) * self.noise])
            new_state.append(self.step_num)
            new_state.append(last_time)
            new_state.append(applied_torque)
            self.output_queue.put(new_state)
            self.output_vals = new_state
            self.log_file.write("wrote " + str(starting_state) + ", " + str(self.current_torque) + " to " +
                                str(new_state) + "id, " + str(self.step_num) + "\n")

            # print(1 / 2 * lower_mass * (body1.getLinearVel()[0] ** 2 + body1.getLinearVel()[1] ** 2) +
            #       1 / 2 * upper_mass * (body2.getLinearVel()[0] ** 2 + body2.getLinearVel()[1] ** 2) +
            #       lower_mass * 9.81 * body1.getPosition()[1] + upper_mass * 9.81 * body2.getPosition()[1])

            # self.output_queue.put(self.last_position)
            self.step_num += 1
            if time.time() < last_time + self.time_step:
                time.sleep(last_time + self.time_step - time.time())

    def close(self):
        self.closed = True
        time.sleep(0.25)
        self.log_file.close()


def constrain(x, a, b):
    if a > x:
        return a
    if b < x:
        return b
    return x
