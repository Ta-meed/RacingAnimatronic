import socket
import struct
import math

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *
"""
Requires PyGame and PyOpenGL

Code replicates the motion of the car with OpenGL for Dirt rally 1 & 2 given 
that the UDP is turned on. It should theoretically work for all code master 
games that use the same UDP format and other games if you know the how to 
extract the pitch, roll, and suspension compression from the game.

Points on the plane
we want to find 4 equally spaced points on the plane
we do this by first getting the cross of the normal of the plane and each
reference vector then normalize each vector

Rotation
we want to find the angle between the pitch vector and the vector (1, 0, 0)
then apply a rotation transformation to a each vector on the plane 

Height
we want to find the maximum compress of the 4 suspensions of the wheels and transform
the vectors accordingly

"""

# Points that make an arrow
reference_vectors = [(-1, 1, 0),
                     (1, 1, 0),
                     (1, -1, 0),
                     (-1, -1, 0),
                     (2, 0, 0)]

# Connection of all the points to make the arrow
edges = [(0,1),
         (1,2),
         (2,3),
         (3,0),
         (1,4),
         (2,4)]

# Unit vector along the x axis
x_vec = (1, 0, 0)

angle_prev = 0
angle_acceleration_prev = 0


def cross(v1, v2):
    """
    The cross product of 2 vectors results in a vector orthogonal to both vectors
    :param v1: The vector left of the cross symbol
    :param v2: The vector right of the cross symbol
    :return: The cross between v1 and v2
    """
    a, b, c = v1
    d, e, f = v2

    return ((b*f-c*e),
            (-a*f+d*c),
            (a*e-d*b))


def magnitude(v):
    """
    Magnitude is a length of a vector
    :param v: The vector we want to find the magnitude of
    :return: The magnitude of v
    """
    return math.sqrt(math.pow(v[0], 2) +
                     math.pow(v[1], 2) +
                     math.pow(v[2], 2))


def unit(v):
    """
    Unit vectors are vectors with magnitude = 1
    :param v: The vector we want to find the unit vector of
    :return: The unit vector of v
    """
    scale = abs(1/magnitude(v))
    return (scale * v[0],
            scale * v[1],
            scale * v[2])


def scale(v, num):
    """
    :param v: The vector we want to scale
    :param num: Scale amount
    :return: vector scaled by num
    """
    a, b, c = v
    return (a*num,
            b*num,
            c*num)


def dot(v1, v2):
    """
    Dot product has many properties
    :param v1: The vector left of the dot symbol
    :param v2: The vector right of the dot symbol
    :return: The dot product between v1 and v2
    """
    a, b, c = v1
    d, e, f = v2
    return a*d + b*e + c*f


def angle_between(v1, v2, n):
    """
    Given that the vectors are on the plane with normal n return the angle between
    them with their signs preserved and an angle up to 360 degrees(2pi)
    :param v1: The first vector
    :param v2: The second vector
    :param n: The normal of the plane
    :return: returns the angle between v1 and v2
    """
    x1, y1, z1 = unit(v1)
    x2, y2, z2 = unit(v2)
    xn, yn, zn = unit(n)

    dotP = dot(v1, v2)
    det = x1 * y2 * zn +\
          x2 * yn * z1 +\
          xn * y1 * z2 -\
          z1 * y2 * xn -\
          z2 * yn * x1 -\
          zn * y1 * x2

    angle = math.atan2(det, dotP)
    return angle


def rotate(v1, angle):
    """
    rotates the vector by an angle around the z axis
    :param v1: The vector that will be rotated
    :param angle: The angle to rotate the vector by
    :return: The vector rotated around the z axis by the angle
    """
    x, y, z = v1

    # Rotation transformation
    v = (x*math.cos(angle) - y*math.sin(angle),
         x*math.sin(angle) + y*math.cos(angle),
         z)

    return v


def adjust_height(v1, height):
    """
    Increases or decreases the z axis of the vector
    :param v1: The vector to have its height changed
    :param height: The height to change the vector by
    :return: The vector with its z parameter summed by height
    """
    return (v1[0],
            v1[1],
            v1[2] + height)


def plane(v1, v2, pitch, height, game_camera=False):
    """
    Transforms arrow to an arrow with based on the car and sets points in OpenGL
    :param v1: roll vector along the y axis
    :param v2: pitch vector along the x axis
    :param pitch: Actual pitch vector of the car
    :param height: height
    :param game_camera: boolean value for if the arrow should act like the
    in-game camera
    """
    n = cross(v1, v2)
    vectors = []
    angle = angle_between(x_vec, pitch, n)


    # angle_prev and angle_acceleration_prev are only used to simulate the
    # way the in-game chase camera works
    global angle_prev
    angle_acceleration = (angle - angle_prev) / 0.07
    angle_prev = angle

    global angle_acceleration_prev
    if angle_acceleration == 0:
        angle_acceleration = angle_acceleration_prev
    else:
        angle_acceleration_prev = angle_acceleration


    # Apply the plane
    for refV in reference_vectors:
        vectors.append(unit(cross(refV, n)))

    # Apply the rotation
    for x in range(len(vectors)):
        if game_camera:
            vectors[x] = rotate(vectors[x], 1/1.2*math.atan(angle_acceleration))
        else:
            vectors[x] = rotate(vectors[x], angle)

    # Apply the height
    for x in range(len(vectors)):
        vectors[x] = adjust_height(vectors[x], height)

    # Make the tip of the arrow longer
    vectors[-1] = scale(vectors[-1], 1.2)

    # Draw everything in OpenGL
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vectors[vertex])
    glEnd()


def main():
    # Initialize to read the data from the socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Create UDP socket.
    sock.bind(('127.0.0.1', 20777))  # Bind to LFS.

    # Initialize the PyGame and OpenGL window
    pygame.init()
    display = (350 + 50, 200 + 50)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(25, (display[0]/display[1]), 0.1, 60.0)
    glTranslatef(0.0, 0.0, -4)
    glRotatef(-80, 1, 0, 0)  # Rotated to look like the chase camera

    while True:
        data = sock.recv(1024)  # Read the socket data
        udp_data = struct.unpack('64f', data[0:256])  # Convert the data to float values

        roll_z = udp_data[12]
        pitch_x = udp_data[14]
        pitch_y = udp_data[16]
        pitch_z = udp_data[15]
        suspension = udp_data[17:21]

        v1 = (1, 0, roll_z)
        v2 = (0, 1, pitch_z)
        pitch = (pitch_x, pitch_y, pitch_z)
        height = max(suspension) * 0.0005

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        plane(v2, v1, pitch, height, True)
        pygame.display.flip()
        pygame.time.wait(0)


main()



