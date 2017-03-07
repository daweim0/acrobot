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


def convert_coord(x, y):
    """Convert world coordinates to pixel coordinates."""
    return int(500 + 170 * x), int(500 - 170 * y)


scr_write_head = 5
font_object = None
loop_flag = True

# Calculate joint positions
l1 = .5
l2 = .75
lower_mass = 1
upper_mass = 1

new_constant_torque = 0.0
position_output_queue = list()


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


# Returns a list containing the passed string divided at the delimiter.
# The delimiter will not be kept in the returned string.
def parse(string, delimiter):
    result = list()
    input_chars = list(string)
    section_number = 0
    for n in range(0, len(input_chars)):
        if input_chars[n] == delimiter:
            section_number += 1
        else:
            if len(result) <= section_number:
                result.append(input_chars[n])
            else:
                result[section_number] += input_chars[n]
    return result


# TODO: work on outer controller (PID maybe?)
# simulation starts here
# def run(starting_q1=math.pi - 50.87 * math.pi / 180,
#         starting_q2=-96.25 * math.pi / 180,
def run(starting_q1=math.pi * 1/2,
        starting_q2=0.1,
        starting_q1v=0.0,
        starting_q2v=0.0,
        q1_friction=None,
        q2_friction=None,
        headless=False,
        fps=20,
        maximum_seconds=9999,
        passive=False,
        constant_torque=0,
        return_state_vector=True,
        has_bounds=False,
        target_state=(math.pi*1/2, 1.0, 0.0, 0.0),
        hopper_weights=None,
        use_ode_as_table=False,
        return_dump=False,
        exit_on_fail=False,
        lookup_parameters=None,
        noise=0.00,
        interactive=False,
        keep_time=False,
        gravity=-9.81):
    if hopper_weights is None:
        # known good tuning parameters without friction:
        hopper_weights = (0.01, 0.0113, 0.0095, 0.01235918, 0.06646608, 0.0134879, 17.04, 0.0389, 0.26385)

        # known good tuning parameter with .25, .1 friction
        # hopper_weights = (0.0100, 0.0113, 0.0091, 0.0098, 0.0860, 0.0094, 18.3585, 0.0389, 0.4468)
        # testing parameters:
        # hopper_weights = (0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00000, 0.0000)

    if lookup_parameters is None:
        lookup_parameters = (3, 10, 1)

    lookup_table_hopper.initialize(42)

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

    if gravity is None:
        gravity = lookup_table_hopper.get_gravity()
    else:
        lookup_table_hopper.set_gravity(gravity)
    gravity_tuple = [0, gravity, 0]

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

    # Create a world object
    world = ode.World()
    world.setGravity(gravity_tuple)
    world.setERP(0.8)  # error correction, .8 is the maximum recommended value
    world.setCFM(0.000000001)  # makes joints slightly springy, helps in setting initial velocities

    # Create two bodies
    body1 = ode.Body(world)
    m1 = ode.Mass()
    m1.setSphereTotal(lower_mass, 0.01)
    body1.setMass(m1)
    body1.setPosition((lower.x(), lower.y(), 0))

    body2 = ode.Body(world)
    m2 = ode.Mass()
    m2.setSphereTotal(upper_mass, 0.01)
    body2.setMass(m2)
    body2.setPosition((upper.x(), upper.y(), 0))

    # Connect body1 with the static environment
    j1 = ode.HingeJoint(world)
    j1.attach(body1, ode.environment)
    j1.setAnchor((origin.x(), origin.y(), 0))
    j1.setAxis((0, 0, 1))

    # Connect body2 with body1
    j2 = ode.HingeJoint(world)
    j2.attach(body1, body2)
    j2.setAnchor((lower.x(), lower.y(), 0))
    j2.setAxis((0, 0, 1))

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
        vel_queue = list()
        balance_curve_list = list()
        with open("balance_curve.csv") as curve:
            while True:
                line = curve.readline()
                if line == "":
                    break
                balance_curve_list.insert(0, (float(parse(line, ",")[0]), float(parse(line, ",")[1])))

    q1 = 0
    q2 = 0
    q1_velocity = 0
    q2_velocity = 0

    # set angular velocities (since ODE has issues...)
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
        current_vel1 = body1.getLinearVel()
        body1.setLinearVel((-1 * v1x, v1y, 0))

        current_vel2 = body2.getLinearVel()
        body2.setLinearVel((-1 * v2x, v2y, 0))

        # For debugging
        # q1_diff = math.atan(v1x/v1y) - q1
        # q2_diff = math.atan((v2x-v1x)/(v2y-v1y)) - (q1 + q2)
        world.step(torque_dt)

        if ((abs(q1_vel[0] - v1x) < accuracy and abs(q1_vel[1] - v1y) < accuracy) and (
                        abs(q2_vel[0] - v2x) < + accuracy and abs(q2_vel[1] - v2y) < accuracy)):
            break

    error_average = 0
    n_errors = 0
    position_list = list()

    world.setCFM(0.0000001)  # Turn joint springiness back down

    def advance_simulation(time_step, torque, ode_world):
        # Next simulation step
        substep_num = 10.0
        for _ in range(int(substep_num)):
            j2.addTorque(-1 * torque)
            # bearing friction
            j2.addTorque(-1 * j2.getAngleRate() * q2_friction)
            j1.addTorque(-1 * j1.getAngleRate() * q1_friction)
            ode_world.step(time_step / substep_num)

    # ####################################################
    # main loop
    # ####################################################

    while loop_flag:
        # Process command input
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

        # pause and allow for terminal input
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
                    break  # no sense in doing another simulation step
                elif command[0] == "fps":
                    fps = int(command[1])
                    dt = 1.0 / fps
                elif command[0] == "gravity":
                    world.setGravity((float(command[1]), 0, 0))
                elif command[0] == "kick":
                    j2.addTorque(float(command[1]) * fps)
            except ValueError:
                print(("ValueError thrown while parsing \"" + str(command) + "\". (was there a mis-formatted float?)"))

                # ################################################
                # simulation stuff
                # ################################################

                # rotate and flip the angles so they agree with how the paper describes them

        def update_measurements():
            """
            returns q1, q2, q1_velocity, q2_velocity
            :return: float, float, float, float
            """
            return j1.getAngle() + (random.random() - .5) * noise + float(starting_q1), -1 * (
                j2.getAngle() + (random.random() - .5) * noise + -1 * float(starting_q2)), j1.getAngleRate() + (
                random.random() - .5) * noise, -1 * j2.getAngleRate() + (random.random() - .5) * noise

        # #################################################################
        # Calculations
        # #################################################################

        if passive and ticks != 0:
            advance_simulation(dt, constant_torque, world)
            print_queue.put("constant torque: " + str(constant_torque))

        elif ticks != 0:
            q1, q2, q1_velocity, q2_velocity = update_measurements()
            if not headless:
                print("elapsed time", str(ticks / float(fps)))
            try:
                final_torque, path, dists = lookup_table_hopper.get_next_torque(
                    [q1, q2, q1_velocity, q2_velocity], target_state, *lookup_parameters, has_bounds=has_bounds)
            except Exception:
                final_torque = 0
                path = [[999, 999, 999, 999, 999], [999, 999, 999, 999]]
                dists = [999, 999]
            if not headless:
                print("path:", path)
            # ode_prediction = lookup_table_generator.run_simulation(q1, q2, q1_velocity, q2_velocity, final_torque,
            #                                                        run_time=1.0 / fps, fps=150)
            # ode_distance = lookup_table_hopper.dist_between(ode_prediction, target_state, torque=final_torque)
            if not headless:
                print("torque applied: ", final_torque)

            advance_simulation(dt, final_torque, world)
            q1, q2, q1_velocity, q2_velocity = update_measurements()

            actual_distance = lookup_table_hopper.dist_between((q1, q2, q1_velocity, q2_velocity), target_state,
                                                               torque=final_torque)
            if not headless:
                print("starting distance = ", dists[0])
                print("predicted ending distance = ", dists[1])
                # print "ODE predicted distance = ", ode_distance, " state:", ode_prediction
                print("actual ending distance = ", actual_distance)
                if dists[1] < actual_distance:
                    print("ended closer than expected (" + str(
                        (dists[1] - actual_distance) / actual_distance * 100) + "% off)")
                else:
                    print("ended further than expected (" + str(
                        (dists[1] - actual_distance) / actual_distance * 100) + "% off)")

                error_average = (error_average * n_errors / (n_errors + 1) +
                                 (abs((dists[1] - actual_distance) / actual_distance) * 100) * 1 / (n_errors + 1))
                n_errors += 1
                print(("average error = " + str(error_average) + "%"))
                if dists[0] < dists[1]:
                    print("can't get closer than current position")
                print("current position = ", path[0])
                print_queue.put("final balancing torque   = " + str('{0:.16f}'.format(final_torque)))
                print()

                # if ticks % fps == 0:  # only execute this once every second
                #     kick_ammount = (q2 - target_state[1])**3 * -1.5
                #     kick(kick_ammount, j2, fps)
                #     print("kicked with " + str(kick_ammount))
                #
                # print("\n\n")

        q1, q2, q1_velocity, q2_velocity = update_measurements()
        position_list.append((q1, q2, q1_velocity, q2_velocity))

        if not headless:
            print_queue.put("angle q1 is = " + str(math.degrees(q1)))
            print_queue.put("angle q2 is = " + str(math.degrees(q2)))
            print_queue.put("q1 velocity = " + str(q1_velocity))
            print_queue.put("q2 velocity = " + str(q2_velocity))

            print_queue.put("fps = " + str(fps) + "  ticks/second")
            print_queue.put("at time " + str(ticks / float(fps)) + "(Seconds)")
            print_queue.put("holding torque = " + str(lookup_table_hopper.get_holding_torque([q1, q2, 0, 0])))
            print_queue.put(
                "center of gravity = " + str("%.10f" % lookup_table_hopper.get_center_of_gravity([q1, q2, 0, 0])))
            print_queue.put(
                "angular momentum = " + str(
                    lookup_table_hopper.get_angular_momentum((q1, q2, q1_velocity, q2_velocity))))
            print_queue.put("energy difference = " + str(lookup_table_hopper.get_mechanical_energy(
                (q1, q2, q1_velocity, q2_velocity)) - lookup_table_hopper.get_mechanical_energy(target_state)))
            print_queue.put(
                "distance = " + str(lookup_table_hopper.dist_between((q1, q2, q1_velocity, q2_velocity), target_state)))

        # in-window plots
        if not headless:
            position_queue.insert(0, (q1, q2))
            vel_queue.insert(0, (q1_velocity, q2_velocity))

        if interactive:
            constant_torque = new_constant_torque
            position_output_queue.append((q1, q2, q1_velocity, q2_velocity))

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
            x1, y1, z1 = body1.getPosition()  # @UnusedVariable
            x2, y2, z2 = body2.getPosition()  # @UnusedVariable

            # draw segments first so that they appear under the circles
            pygame.draw.line(screen, (55, 0, 200), convert_coord(origin.x(), origin.y()), convert_coord(x1, y1), 2)
            pygame.draw.line(screen, (55, 0, 200), convert_coord(x1, y1), convert_coord(x2, y2), 2)

            # draw the origin
            pygame.draw.circle(screen, (0, 100, 100), convert_coord(int(origin.x()), int(origin.y())), 10, 0)

            # draw the lower section
            pygame.draw.circle(screen, (0, 100, 100), convert_coord(x1, y1), 10, 0)

            # draw the upper section
            pygame.draw.circle(screen, (0, 100, 100), convert_coord(x2, y2), 10, 0)

            # draw graphs
            plot1_origin = [100, 500]
            scale = 30
            n_samples = 200
            for i in balance_curve_list:
                cord = (int(plot1_origin[0] + i[0] * scale), int(plot1_origin[1] + i[1] * scale))
                pygame.draw.circle(screen, (150, 0, 150), cord, 2, 0)
            for i in range(len(position_queue) - 1, -1, -1):
                cord = (int(plot1_origin[0] + position_queue[i][0] * scale),
                        int(plot1_origin[1] + position_queue[i][1] * scale))
                pygame.draw.circle(screen, (0, 200 - i, 0), cord, 4, 0)
            if len(position_queue) > n_samples:
                position_queue.pop()
            # write debug information
            reset_screen_print()
            screen_print("acrobot simulation")
            while not print_queue.empty():
                screen_print(print_queue.get())

            # stop if out of time
            if float(maximum_seconds) * float(fps) < ticks:
                print("ran out of time, limit was " + str(maximum_seconds) + " seconds, " + str(ticks) + " ticks")
                loop_flag = False

            # update display
            pygame.display.flip()

        if not headless or keep_time:
            # Try to keep the specified frame rate
            clk.tick(fps)

        # advance simulation counter
        ticks += 1

        if maximum_seconds * fps < ticks or (
            exit_on_fail and (body1.getPosition()[1] < 0 or body2.getPosition()[1] < 0)):
            # print "ran out of time, limit was " + str(maximum_seconds) + " seconds"
            loop_flag = False

    if not headless:
        pygame.display.quit()
        screen = None
        font_object = None
    # put together return values
    if return_state_vector:
        return float(q1), float(q2), float(q1_velocity), float(q2_velocity)
    if return_dump:
        return float(q1), float(q2), float(q1_velocity), float(q2_velocity), float(ticks) / fps, position_list


def set_constant_torque(torque):
    global new_constant_torque
    new_constant_torque = torque


def get_holding_torque(tupl):
    """
    Returns the torque required to counteract gravity on the upper segment.
    Args:
        tupl (float, float, float, float): acrobot position (angles)
    Returns:
        float: torque required
    """
    return lower_mass * l2 * 9.8 * math.cos(tupl[0] + tupl[1])


def kick(dl, j2, fps):
    """
    Applies dl*fps torque to j2 for 1 tick (equivalent of applying dl torque for 1 second).
    :param dl: float
    :param j2: object
    :param fps: int
    """
    j2.addTorque(dl * fps)


if __name__ == "__main__":
    run()
