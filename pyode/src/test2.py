# pyODE example 2: Connecting bodies with joints

import pygame
from pygame.locals import *
import ode


def coord(x,y):
    """Convert world coordinates to pixel coordinates."""
    return 320+170*x, 400-170*y


# Initialize pygame
pygame.init()

# Open a display
srf = pygame.display.set_mode((640, 480))

# Create a world object
world = ode.World()
world.setGravity((0, -9.81, 0))
world.setERP(0.8)
world.setCFM(0.000001)

# Create two bodies
body1 = ode.Body(world)
M = ode.Mass()
M.setSphere(2500, 0.05)
body1.setMass(M)
body1.setPosition((0, 1, 0))

body2 = ode.Body(world)
M = ode.Mass()
M.setSphere(2500, 0.05)
body2.setMass(M)
body2.setPosition((0, 0, 0))

# Connect body1 with the static environment
j1 = ode.HingeJoint(world)
j1.attach(body1, ode.environment)
j1.setAnchor((0, 2, 0))
j1.setAxis((0, 0, 1))

j1.setParam(ode.ParamVel, 3)
j1.setParam(ode.ParamFMax, 22)

# Connect body2 with body1
j2 = ode.HingeJoint(world)
j2.attach(body1, body2)
j2.setAnchor((0, 1, 0))
j2.setAxis((0, 0, 1))


# Simulation loop...

fps = 200000
dt = 1.0/fps
loopFlag = True
clk = pygame.time.Clock()

while loopFlag:
    events = pygame.event.get()
    for e in events:
        if e.type == QUIT:
            loopFlag = False
        if e.type == KEYDOWN:
            loopFlag = False

#     j1.addTorque(20)

    # Clear the screen
    srf.fill((255, 255, 255))

    # Draw the two bodies
    x1, y1, z1 = body1.getPosition()
    x2, y2, z2 = body2.getPosition()
    pygame.draw.circle(srf, (55, 0, 200), coord(x1, y1), 20, 0)
    pygame.draw.line(srf, (55, 0, 200), coord(0, 2), coord(x1, y1), 2)
    pygame.draw.circle(srf, (55, 0, 200), coord(x2, y2), 20, 0)
    pygame.draw.line(srf, (55, 0, 200), coord(x1, y1), coord(x2, y2), 2)

    pygame.display.flip()

    # Next simulation step
    j2.addTorque(50)
    world.step(dt)

    # Try to keep the specified framerate    
    clk.tick(fps)