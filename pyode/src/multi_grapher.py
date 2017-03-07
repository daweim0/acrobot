"""
Created on Aug 7, 2014

@author: David
"""

import math
import os
import matplotlib.pyplot as plt
import simulation
import sys


def parse(string, delimiter):
    result = list()
    input = list(string)
    section_number = 0
    for i in range(0, len(input)):
        if input[i] == delimiter:
            section_number += 1
        else:
            if len(result) <= section_number:
                result.append(input[i])
            else:
                result[section_number] += input[i]
    return result


def emptydir(top):
    if top == '/' or top == "\\":
        return
    else:
        for root, dirs, files in os.walk(top, topdown=False):
            for name in files:
                os.remove(os.path.join(root, name))
            for name in dirs:
                os.rmdir(os.path.join(root, name))


def read_file(name, has_header=True):
    next_file = open(name, "r")
    file_points = list()

    # remove headder line
    if has_header:
        next_file.readline()
    FileNotDone = True
    while FileNotDone:
        line = next_file.readline()
        if line == "":
            FileNotDone = False
        else:
            file_points.append(parse(line, ","))
    return file_points


def run(headless, friction, save_name, threaded_terminal, logging_dir, maxTime, constant_torque,
        overlay_log, starting_torque, parent_log):
    # if len(sys.argv) > 1:
    # headless = True
    # save_name = sys.argv[1]
    #         friction = float(sys.argv[2])

    q1_index = 1
    q2_index = 2

    x_index = 0
    y_index = 1

    # read parent next_file

    points = read_file(parent_log, has_header=False)
    i = 0

    # comment out next for loop to use already existing logs
    emptydir(logging_dir)

    # loop and run each simulation
    points.pop(0)
    for point in points:
        if not (point[0] == "" or point[0] == "\n"):
            print(point)
            if len(point) >= 3 and len(point[q1_index]) >= 2:
                i += 1
                print("starting simulation " + str(i) + "x1, x2, " + str(point[q1_index]) + " " + str(
                    point[q2_index]))

                simulation.run(starting_q1v=0, starting_q2v=0, starting_damp1=friction, starting_damp2=0,
                               headless=True, terminal=threaded_terminal, log_from_beginning=True,
                               logger_dir=logging_dir, fps=32, exit_on_fail=True, maximum_seconds=.25,
                               starting_goto=0, passive=False, constant_torque=constant_torque,
                               starting_q2=1 * float(point[q2_index]), starting_q1=float(point[q1_index]),
                               starting_torque=0)
            # except:
            #     print('simulation cut off by exception')

    print("loading graph")

    # load all simulations
    file_names = os.listdir(os.getcwd() + "/" + logging_dir)
    print("reading logs " + str(file_names))

    logs = list()
    for logFile in file_names:
        next_file = open(logging_dir + "/" + logFile, "r")
        next_file.readline()
        log_points = list()
        while True:
            line = next_file.readline()
            if not line == "":
                log_points.append(parse(next_file.readline(), ","))
            else:
                break

        logs.append(log_points)

    skip_every = 5
    color_index = 0

    colors = "bgcmy"

    counter = 0
    colorCounter = 0

    overlay = None

    if overlay_log != "":
        overlay = list()
        next_file = open(overlay_log, "r")
        next_file.readline()
        log_points = list()
        while True:
            line = next_file.readline()
            if not line == "":
                overlay.append(parse(next_file.readline(), ","))
            else:
                break

    color_index = 0
    for logFile in logs:
        x = list()
        y = list()
        for n_line in range(0, len(logFile)):
            if len(logFile[n_line]) >= 3 and len(logFile[n_line][x_index]) >= 2:
                plt.scatter(float(logFile[n_line][x_index]), float(logFile[n_line][y_index]),
                            edgecolor=colors[color_index], c=colors[color_index], s=3)
        color_index = (color_index + 1) % len(colors)

        # faster than above section, but doesn't allow for multicolor
        #                 x.append(float(logFile[n_line][x_index]))
        #                 y.append(float(logFile[n_line][y_index]))
        plt.plot(x, y, "b.")
        print("loaded log")

    # add theoretical curve
    if overlay is not None:
        for point in overlay:
            if len(point) > 2:  # make sure lines aren't excel created scruff
                #                     print " x index, y index " + str(point[q1_index]) + " " + str(point[q2_index])
                plt.scatter(float(point[q1_index]), float(point[q2_index]), edgecolor="r", c="r", s=3)

    plt.axis([-.2, math.pi + .2, (-1 * math.pi) - .2, math.pi + .2])
    if not headless:
        plt.show()
    if headless:
        plt.title("torque = " + str(constant_torque) + "   friction = " + str(friction))
        plt.xlabel("q1")
        plt.ylabel("q2")
        plt.gca().invert_xaxis()
        plt.savefig(save_name + str(file_names[counter]) + ".png", bbox_inches='tight')
        counter += 1
    plt.clf()

    #     plt.plot(-2,-2)

    print("finished loading")

# plt.axis([-.1, math.pi + .1,(-1 * math.pi) - .1, math.pi + .1])
# if not headless:
# plt.show()
#     if headless:
#         plt.savefig(save_name, bbox_inches='tight')


if __name__ == '__main__':
    run(False, 0, "passive_right", True, "logging_test", 10, 0, "parent_log_balance_curve.csv", 0,
        "parent_log_balance_curve_right.csv")


#  headless   friction   save_name   threadded_terminal   logging_dir   maximum time 
#  torque   overlay file   starting torque   parent log





