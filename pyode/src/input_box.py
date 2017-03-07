# by Timothy Downs, inputbox written for my map editor

# This program needs a little cleaning up
# It ignores the shift key
# And, for reasons of my own, this program converts "-" to "_"

# A program to get user input, allowing backspace etc
# shown in a box in the middle of the screen
# Called by:
# import inputbox
# answer = inputbox.ask(screen, "Your name")
#
# Only near the center of the screen is blitted to

import pygame, pygame.font, pygame.event, pygame.draw, string, time
from pygame.locals import *


def get_key():
    while 1:
        event = pygame.event.poll()
        if event.type == KEYDOWN:
            return event.key
        else:
            pass


def display_box(screen, message):
    """Print a message in a box in the middle of the screen"""
    fontobject = pygame.font.Font(None, 18)
    pygame.draw.rect(screen, (0, 0, 0),
                     ((screen.get_width() / 2) - 400,
                      (screen.get_height() / 2) + 200,
                      200, 20), 0)
    pygame.draw.rect(screen, (255, 255, 255),
                     ((screen.get_width() / 2) - 402,
                      (screen.get_height() / 2) + 198,
                      204, 24), 1)
    if len(message) != 0:
        screen.blit(fontobject.render(str(message), 1, (255, 255, 255)),
                    ((screen.get_width() / 2) - 400, (screen.get_height() / 2) + 200))
    pygame.display.flip()


def ask(screen, question):
    """ask(screen, question) -> answer"""
    pygame.font.init()
    current_string = ''
    display_box(screen, question + ": " + str(current_string))
    while 1:
        inkey = get_key_local()
        if inkey == K_BACKSPACE:
            current_string = current_string[0:-1]
        elif inkey == K_RETURN:
            break
        elif inkey <= 127:
            current_string += str(chr(inkey))
        display_box(screen, question + ": " + str(current_string))
    return str(current_string)


def get_key_local():
    while 1:
        event = pygame.event.poll()
        if event.type == KEYDOWN:
            return event.key
        time.sleep(.02)  # to keep python from contributing to global warming


# else:
# 			time.sleep(.2)
# 			if pygame.key.get_pressed()[K_BACKSPACE]:
# 				return K_BACKSPACE


def main():
    screen = pygame.display.set_mode((320, 240))
    print(ask(screen, "Name") + " was entered")


if __name__ == '__main__': main()
