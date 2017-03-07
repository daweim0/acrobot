'''
Created on Nov 24, 2014
@author: David Michleman
'''
import math
import time
import sys
import pygame
from pygame.locals import QUIT
from ast import literal_eval as make_tuple


def display(path):
    if type(path) == str:
        path = make_tuple(path)

    pygame.init()
    screen = pygame.display.set_mode((900, 600))
    screen.fill((0, 250, 250))

    for i in range(0, len(path)):
        if type(path[i][0]) != tuple:
            for j in range(0, 1):
                draw_state(screen, path[i][j], int(map_to(i, 0, len(path), 0, 255)) + j * 3)

    pygame.display.flip()

    event = None
    while event is None or len(event) == 0 or event[0].type != QUIT:
        time.sleep(.01)
        event = pygame.event.get()
    pygame.quit()
    sys.exit()


def draw_state(screen, point, hue):
    q1_index = 0
    q2_index = 1
    l1 = 0.5
    l2 = 0.75
    x1 = math.cos(float(point[q1_index])) * l1
    y1 = math.sin(float(point[q1_index])) * l1
    x2 = x1 + math.cos((float(point[q1_index]) + float(point[q2_index]))) * l2
    y2 = y1 + math.sin((float(point[q1_index]) + float(point[q2_index]))) * l2

    scale_factor = 300

    draw_joint(screen, 0, 50, hue=hue)
    draw_joint(screen, 0 + x1 * scale_factor, 50 + y1 * scale_factor, hue=hue)
    draw_joint(screen, 0 + x2 * scale_factor, 50 + y2 * scale_factor, hue=hue)


def draw_joint(screen, x, y, hue=0):
    pygame.draw.rect(screen, (hue, hue, hue), (450 + x, 600 - y, 20, 20), 5)


def map_to(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


if __name__ == '__main__':
    path = "[[1.5707963267948966, -0.22439947525641379, 0.0, 0.0, 0], [1.5731204311039786, -0.23491707245490012, " \
           "0.065097466111183167, 0.25337463617324829, 0], [1.5770371715214591, -0.70179291963520818, " \
           "0.15050938725471497, 0.65296876430511475, 0], [3.0082171729174698, 0.92307476679975908, " \
           "4.2060364335775375, 11.10960865020752]]"
    display(path)