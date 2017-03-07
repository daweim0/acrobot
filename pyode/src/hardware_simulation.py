import math
import queue
import time
import numpy
import pygame
import pygame.locals  # @UnusedWildImport
import random
import ode
import input_box
import lookup_table_hopper
import lookup_table_generator
import hardware
import ode_interface
import hardware_lookup_table_generator
from subprocess import call
from collections import deque
from acrobot_state import State


position_output_queue = list()


def run(starting_q1=math.pi * 3 / 2,
        starting_q2=0,
        starting_q1v=0.0,
        starting_q2v=0.0,
        q1_friction=50.0,
        q2_friction=0.0,
        headless=False,
        fps=5,
        maximum_seconds=9999,
        passive=False,
        constant_torque=0,
        return_state_vector=True,
        target_state=(math.pi / 2 + 0.0, 0.0, 0.0, 0.0),
        hopper_weights=None,
        use_ode_as_table=False,
        return_dump=False,
        lookup_parameters=None,
        keep_time=False,
        use_hardware=True,
        l1=0.5,
        l2=0.75,
        lower_mass = 1,
        upper_mass = 1,
        gravity=1.0,
        systematic_torques = True):
    """
    :param starting_q1:
    :param starting_q2:
    :param starting_q1v:
    :param starting_q2v:
    :param q1_friction:
    :param q2_friction:
    :param headless:
    :param fps:
    :param maximum_seconds:
    :param passive:
    :param constant_torque:
    :param return_state_vector:
    :param has_bounds:
    :param target_state:
    :param hopper_weights:
    :param use_ode_as_table:
    :param return_dump:
    :param lookup_parameters:
    :param interactive:
    :param keep_time:
    :param use_hardware:
    :param l1:
    :param l2:
    :param lower_mass:
    :param upper_mass:
    :param gravity: simulated gravity
    :return:
    """
    # TODO: fill out docstring
    if hopper_weights is None:
        # known good tuning parameters without friction:
        # hopper_weights = (0.76075, 0.08325, 0.10000, 0.95721, 0.10000, 0.05704, 10.41774, 0.00000, 0.10000)

        # known good tuning parameter with .25, .1 friction
        # hopper_weights = (0.0100, 0.0113, 0.0091, 0.0098, 0.0860, 0.0094, 18.3585, 0.0389, 0.4468)
        # testing parameters:
        hopper_weights = (0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00000, 0.0000)

    if lookup_parameters is None:
        lookup_parameters = (1, 7, 1)

    if not passive:
        lookup_table_hopper.set_weights(hopper_weights)

    if fps is None:  # make fps same as in lookup table
        fps = lookup_table_hopper.get_fps()

    if q1_friction is None:
        q1_friction = lookup_table_hopper.get_q1_friction()

    if q2_friction is None:
        q2_friction = lookup_table_hopper.get_q2_friction()

    if use_ode_as_table is not None:
        lookup_table_hopper.use_ode_as_table(use_ode_as_table)

    if use_hardware:
        last_hardware_positions = list([math.pi / 2, 0, 0, 0])
        robot = hardware.acrobot()
        robot.stop()
    else:
        last_hardware_positions = list([math.pi / 2, 0, 0, 0])
        robot = ode_interface.acrobot(gravity_tuple=[0, gravity, 0], q1_friction=q1_friction, q2_friction=q2_friction)
        robot.stop()

    table_generator = hardware_lookup_table_generator.LookupTableFiller(use_simulation=not use_hardware)
    lookup_table_hopper.initialize(None, imported_table=table_generator.table)
    lookup_table_hopper.set_parameters(q1_low_=table_generator.q1_low,
                                       q1_high_=table_generator.q1_high,
                                       q2_low_=table_generator.q2_low,
                                       q2_high_=table_generator.q2_high,
                                       q1_vel_low_=table_generator.q1_vel_low,
                                       q1_vel_high_=table_generator.q1_vel_high,
                                       q2_vel_low_=table_generator.q2_vel_low,
                                       q2_vel_high_=table_generator.q2_vel_high,
                                       torque_low_=table_generator.torque_low,
                                       torque_high_=table_generator.torque_high,
                                       resolution_=table_generator.resolution,
                                       velocity_resolution_=table_generator.velocity_resolution,
                                       torque_resolution_=table_generator.torque_resolution)
    print("lookup table entries: " + str(len(numpy.argwhere(table_generator.table)) / 4))

    starting_y1 = math.sin(float(starting_q1)) * l1
    starting_x1 = math.cos(float(starting_q1)) * l1
    starting_x2 = starting_x1 + math.cos((float(starting_q1) + float(starting_q2))) * l2
    starting_y2 = starting_y1 + math.sin((float(starting_q1) + float(starting_q2))) * l2

    origin = Cord(0, 0)
    lower = Cord(origin.x() + starting_x1, origin.y() + starting_y1)
    upper = Cord(origin.x() + starting_x2, origin.y() + starting_y2)

    # queue to print window output to (some declarations use it, so it must be set up early)
    print_queue = queue.Queue()

    # Initialize pygame
    if not headless:
        pygame.init()
        global screen
        screen = pygame.display.set_mode((960, 800))

    # Simulation loop setup
    dt = 1.0 / fps
    global loop_flag
    loop_flag = True
    clk = pygame.time.Clock()

    ticks = 0

    # user input console code
    stepping = False
    paused = False  # also defined here to prevent warnings saying paused might not be defined later on
    if stepping:
        paused = True

    # screen printing
    if not headless:
        global font_object
        font_object = pygame.font.Font(None, 23)

        # in-window graphing stuff
        position_queue = list()
        torque_queue = deque()
        balance_curve_list = list()
        with open("balance_curve.csv") as curve:
            while True:
                line = curve.readline()
                if line == "":
                    break
                balance_curve_list.insert(0, (float(line.split(',')[0]), float(line.split(',')[1])))
            curve.close()

    q1 = 0
    q2 = 0
    q1_velocity = 0
    q2_velocity = 0

    error_total = 0.0
    error_percentage = 0.0
    n_errors = 1
    position_list = list()

    start_time = time.time()
    last_time_fps_counter = start_time
    current_time = start_time - dt
    fps_average = fps
    last_applied_torque = 0.0

    # ####################################################
    # main loop
    # ####################################################

    while loop_flag:
        # Check for key presses
        if not headless:
            events = pygame.event.get()
            for e in events:
                if e.type == pygame.locals.QUIT:
                    loop_flag = False
                if e.type == pygame.K_ESCAPE:
                    loop_flag = False
                if e.type == pygame.locals.KEYDOWN:
                    key = pygame.key.get_pressed()
                    if key[13]:  # enter key
                        paused = True
                    else:
                        print(key)
                if e.type == pygame.locals.KEYUP:
                    pass

        # Accept command inputs
        if paused and not headless:
            # set up input box
            command = input_box.ask(screen, 'Message')
            try:
                command = command.split(" ")
                if command[0] == "":
                    if not stepping:
                        paused = False
                elif command[0] == "step" or command[0] == "s":
                    if command[1] == "true" or command[1] == "y" or command[1] == "on":
                        stepping = True
                        paused = True
                        print("stepping on")
                    else:
                        stepping = False
                        paused = False
                elif command[0] == "exit":
                    break  # don't do another simulation step
                elif command[0] == "fps":
                    fps = int(command[1])
                    dt = 1.0 / fps
                elif command[0] == "stop":
                    loop_flag = False
                elif command[0] == 'passive':
                    passive = True
                    try:
                        if command[1] == 'false' or command[1] == 'off':
                            passive = False
                    except Exception:
                        pass
            except ValueError:
                print(("ValueError thrown while parsing \"" + str(command) + "\". (was there a mis-formatted float?)"))

        # #################################################################
        # Calculations
        # #################################################################

        last_time = current_time
        current_time = time.time()
        led_color = 'green'

        if passive:
            # set torque to constant_torque
            next_torque = constant_torque

        if not -90 / 180 * math.pi < q2 < 90 / 180 * math.pi:
            if 0 < q2:
                next_torque = -50
            else:
                next_torque = 50
            led_color = 'red'
            print('out of bounds')
            filling_new_torque = False
        else:
            # check if the lookup table needs to be filled
            lookup_table_torque = table_generator.get_torque((q1, q2, q1_velocity, q2_velocity))
            if lookup_table_torque is not None:
                next_torque = lookup_table_torque
                led_color = 0x03
                filling_new_torque = True

            else:
                if systematic_torques:
                    next_torque = lookup_table_hopper.map_to(random.random(), 0, 1, lookup_table_hopper.torque_low,
                                                             lookup_table_hopper.torque_high)
                    led_color = 'blue'
                    filling_new_torque = False
                else:
                    # the lookup table does not need to be filled
                    # TODO: The table's dt might be different from the time get_next_torque() takes. Can this be scaled for?
                    future_position = lookup_table_hopper.get_next_angles_interpolated([q1, q2, q1_velocity, q2_velocity, last_applied_torque])
                    if future_position[0] > 100:  # this signifies that there wasn't enough data to do the interpolation or that there was an error
                        hopper_torque = 0
                    else:
                        hopper_torque, path, dists = lookup_table_hopper.get_next_torque(future_position, target_state, *lookup_parameters)
                    next_torque = hopper_torque
                    led_color = 'blue'
                    filling_new_torque = False

        current_acrobot_state = robot.write_read(next_torque)
        robot.set_led(led_color)
        current_acrobot_state[0] %= math.pi*2
        current_acrobot_state.angles[4] = next_torque
        table_generator.feed_data(current_acrobot_state)
        q1, q2, q1_velocity, q2_velocity = current_acrobot_state.angles[0:4]
        last_applied_torque = next_torque

        # table_generator.test_get_torque_and_iterate_helper()

        # TODO: implement error
        error = 0

        position_output_queue.append((q1, q2, q1_velocity, q2_velocity))
        position_list.insert(0, (q1, q2, q1_velocity, q2_velocity, next_torque, error, filling_new_torque))

        if not headless:
            print_queue.put("angle q1 is = " + str(math.degrees(q1)) + " (" + str(q1) + ")")
            print_queue.put("angle q2 is = " + str(math.degrees(q2)) + " (" + str(q2) + ")")
            print_queue.put("q1 velocity = " + str(q1_velocity))
            print_queue.put("q2 velocity = " + str(q2_velocity))

            print_queue.put("final torque   = " + str('{0:.16f}'.format(next_torque)))

            fps_average = fps_average * .75 + 0.25 / (current_time - last_time)
            print_queue.put("fps = " + str(fps_average))
            print_queue.put("at time " + str(current_time - start_time) + " (Seconds)")
            try:
                print_queue.put("average error = " + str(table_generator.accuracy_average / table_generator.average_n))
            except ZeroDivisionError:
                pass
            # print_queue.put("holding torque = " + str(lookup_table_hopper.get_holding_torque([q1, q2, 0, 0])))
            # print_queue.put(
            #     "center of gravity = " + str("%.10f" % lookup_table_hopper.get_center_of_gravity([q1, q2, 0, 0])))
            # print_queue.put(
            #     "angular momentum = " + str(
            #         lookup_table_hopper.get_angular_momentum((q1, q2, q1_velocity, q2_velocity))))
            # print_queue.put("energy difference = " + str(lookup_table_hopper.get_mechanical_energy(
            #     (q1, q2, q1_velocity, q2_velocity)) - lookup_table_hopper.get_mechanical_energy(target_state)))
            print_queue.put(
                "distance = " + str(lookup_table_hopper.dist_between((q1, q2, q1_velocity, q2_velocity), target_state)))

        # ################################################
        # graphics
        # ################################################

        if not headless:
            # Clear the screen
            screen.fill((255, 255, 255))

            # draw grid
            for i in range(-3, 5):
                pygame.draw.line(screen, (0x9F, 0x9F, 0x9F), convert_coord(i, 6), convert_coord(i, -6), 1)
                pygame.draw.line(screen, (0x9F, 0x9F, 0x9F), convert_coord(6, i), convert_coord(-6, i), 1)

            # get the coordinate of the two bodies
            x1 = math.cos(q1) * l1
            x2 = math.cos(q1 + q2) * l2 + x1
            y1 = math.sin(q1) * l1
            y2 = math.sin(q1 + q2) * l2 + y1

            # draw segments first so that they appear under the circles
            pygame.draw.line(screen, (55, 0, 200), convert_coord(origin.x(), origin.y()), convert_coord(x1, y1), 3)
            pygame.draw.line(screen, (55, 0, 200), convert_coord(x1, y1), convert_coord(x2, y2), 3)

            # draw the origin
            pygame.draw.circle(screen, (0, 100, 100), convert_coord(int(origin.x()), int(origin.y())), 10, 0)

            # draw the lower section
            pygame.draw.circle(screen, (0, 100, 100), convert_coord(x1, y1), 10, 0)

            # draw the upper section
            pygame.draw.circle(screen, (0, 100, 100), convert_coord(x2, y2), 10, 0)

            # draw graphs
            # plot 1 (position)
            plot1_origin = [100, 500]
            scale = 30
            n_samples = 200
            for i in balance_curve_list:
                cord = (int(plot1_origin[0] + i[0] * scale), int(plot1_origin[1] + i[1] * scale))
                pygame.draw.circle(screen, (150, 0, 150), cord, 2, 0)
            for i in range(len(position_list) - 1, -1, -1):  # plot in reverse so newest points are on top
                cord = (int(plot1_origin[0] + position_list[i][0] * scale),
                        int(plot1_origin[1] + position_list[i][1] * scale))
                # make the first circle a different color
                if i == 0:
                    pygame.draw.circle(screen, (200, 50, 0), cord, 6, 0)
                else:
                    if position_list[i][2] is not None:
                        if position_list[i][2] == "absent":
                            pygame.draw.circle(screen, (0, 0, 200), cord, 4, 0)
                        else:
                            color = clamp(200 - abs(position_list[i][5]) * 200, 0, 200)
                            pygame.draw.circle(screen, (200 - color, color, 0), cord, 4, 0)
                    else:
                        pygame.draw.circle(screen, (100, 100, 100), cord, 4, 0)

            # plot 2 (torques)
            plot2_origin = [50, 700]
            scale2x = 3
            scale2y = 40 / (abs(lookup_table_hopper.torque_high) + abs(lookup_table_hopper.torque_low))
            n2_samples = 200
            for i in range(len(position_list) - 1):
                cord = (int(plot2_origin[0] + i * scale2x),
                        int(plot2_origin[1] + position_list[i][4] * scale2y * -1))
                # make the first few circles a different color
                if position_list[i][4] is None:
                    pygame.draw.circle(screen, (200, 200, 200), cord, 4, 0)
                elif position_list[i][6]:
                    pygame.draw.circle(screen, (0, 0, 200), cord, 3, 0)
                else:
                    color_num = position_list[i][4] * 300
                    color_num = abs(color_num)
                    color_num = clamp(color_num, 0, 200)
                    pygame.draw.circle(screen, (color_num, 250 - color_num, 0), cord, 3, 0)
            pygame.draw.line(screen, (0, 0, 0), (plot2_origin[0], plot2_origin[1] - 1),
                             (plot2_origin[0] + scale2x * n2_samples, plot2_origin[1] - 1), 2)

            if len(position_list) > n_samples:
                position_list.pop()

            # write debug information
            reset_screen_print()
            screen_print("acrobot simulation")
            while not print_queue.empty():
                screen_print(print_queue.get())

            # update display
            pygame.display.flip()

        if not headless or keep_time:

            while last_time_fps_counter + 1.0 / fps > time.time():
                pass
                time.sleep(0.005)
            last_time_fps_counter = time.time()
        # advance simulation counter
        ticks += 1

        if time.time() - maximum_seconds > start_time:
            print("ran out of time, limit was " + str(maximum_seconds) + " seconds, " + str(ticks) + " ticks")
            loop_flag = False

    # close
    robot.stop()
    robot.exit()
    del robot
    robot = None
    table_generator.close()

    if not headless:
        pygame.display.quit()
        screen = None
        font_object = None
    # put together return values
    if return_state_vector:
        return float(q1), float(q2), float(q1_velocity), float(q2_velocity)
    if return_dump:
        return float(q1), float(q2), float(q1_velocity), float(q2_velocity), float(ticks) / fps, position_list


# def get_holding_torque(tupl: [float, float, float, float]) -> float:
#     """
#     Returns the torque required to counteract gravity on the upper segment.
#     Args:
#         tupl (float, float, float, float): acrobot position (angles)
#     Returns:
#         float: torque required
#     """
#     return lower_mass * l2 * 9.8 * math.cos(tupl[0] + tupl[1])


def kick(dl, j2, fps):
    """
    Applies dl*fps torque to j2 for 1 tick (equivalent of applying dl torque for 1 second).
    :param dl: float
    :param j2: object
    :param fps: int
    """
    j2.addTorque(dl * fps)


def error_percent(a, b):
    raise AssertionError('error_precent should not be used')
    return math.tanh(abs(((a + b) / 2.0) - a) / 10.0) * 100


def convert_coord(x, y):
    """Convert world coordinates to pixel coordinates."""
    return int(500 + 170 * x), int(500 - 170 * y)


scr_write_head = 5
font_object = None
loop_flag = True


def reset_screen_print():
    global scr_write_head
    scr_write_head = 5


# prints values to the screen instead of terminal
def screen_print(string):
    global scr_write_head
    screen.blit(font_object.render(string, 1, (0, 0, 0)), (10, scr_write_head))
    scr_write_head += 23


def do_slopes():
    pass


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


def clamp(n, min_n, max_n):
    return max(min(max_n, n), min_n)


if __name__ == "__main__":
    for i in range(1):
        run(maximum_seconds=99999)
        # call(["python", "lookup_table_checker.py", "save"])
        time.sleep(5)
        # call(["./stop"])
