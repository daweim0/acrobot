import numpy as np
import math

cimport numpy as np
cimport cython

from libc.math cimport floor, sin, cos, acos, M_PI, sqrt

@cython.boundscheck(False)
@cython.wraparound(False)
def get_next_angles_interpolated(np.ndarray[np.float32_t, ndim=6] table, tupl):
    """Gets and returns an interpolated set of angles from a table index. Extrapolation will be performed if the passed
    index is outside of the loaded table. It should be passed lookup table indices, and it will return angles.
    :param tupl: [float, float, float, float, float]
    :return: [float, float, float, float]
    """

    cdef double tupl0 = <double> tupl[0]
    cdef double tupl1 = float(tupl[1])
    cdef double tupl2 = float(tupl[2])
    cdef double tupl3 = float(tupl[3])
    cdef double tupl4 = float(tupl[4])
    cdef int q1a, q1b, q2a, q2b, v1a, v1b, v2a, v2b, ta, tb

    q1a = <int> floor(tupl0)

    q2a = <int> floor(tupl1)

    v1a = <int> floor(tupl2)

    v2a = <int> floor(tupl3)

    ta = <int> floor(tupl4)

    q1a = fixbounds(q1a, 0, table.shape[0]-1)
    q2a = fixbounds(q2a, 0, table.shape[1]-1)
    v1a = fixbounds(v1a, 0, table.shape[2]-1)
    v2a = fixbounds(v2a, 0, table.shape[3]-1)
    ta = fixbounds(ta, 0, table.shape[4]-1)

    # print "q1", q1a, q1b
    # print "q2", q2a, q2b
    # print "v1", v1a, v1b
    # print "v2", v2a, v2b
    # print "t ", ta, tb

    q1b = q1a + 1
    q2b = q2a + 1
    v1b = v1a + 1
    v2b = v2a + 1
    tb = ta + 1

    # print "q1", q1a, q1b
    # print "q2", q2a, q2b
    # print "v1", v1a, v1b
    # print "v2", v2a, v2b
    # print "t ", ta, tb
    #
    # print "\n\n"


    # take a cube out of the larger table surrounding the point in question
    cdef int a, b, c, d, e, f, count, digit
    cdef double tmpa, tmpb, tmpc, tmpd, tmpf, tmp, temp, result
    cdef int i, q1, q2, v1, v2, t, t1, t2, flip_temp

    cdef int adjust[5]
    for i in range(5):
        adjust[i] = 0


    # fill_adjust_aray(adjust, q1a, q2a, ta, table, v1a, v2a)

    cdef double mini[2][2][2][2][2][4]
    # mini = np.ndarray([2, 2, 2, 2, 2, 4], dtype=np.float32)
    # for a in range(2):
    #     for b in range(2):
    #         for c in range(2):
    #             for d in range(2):
    #                 for e in range(2):
    #                     for f in range(4):
    #                         mini[a][b][c][d][e][f] = <float> table[q1a+a, q2a+b, v1a+c, v2a+d, ta+e, f]

    # now interpolate within the mini array
    output = list()

    # interpolate first 4 dimensions getting rid of dimension 5
    for i in range(4):
        for q1 in range(0, 2):
            for q2 in range(0, 2):
                for v1 in range(0, 2):
                    for v2 in range(0, 2):
                        if -0.0001 < table[q1a+q1, q2a+q2, v1a+v1, v2a+v2, ta + 1, i] < 0.0001:
                            mini[q1][q2][v1][v2][0][i] = <float> table[q1a+q1, q2a+q2, v1a+v1, v2a+v2, ta, i]
                            # print "first"
                            # return [999, 999, 999 ,6667]
                        elif -0.0001 < table[q1a+q1, q2a+q2, v1a+v1, v2a+v2, ta, i] < 0.0001:
                            mini[q1][q2][v1][v2][0][i] = <float> table[q1a+q1, q2a+q2, v1a+v1, v2a+v2, ta + 1, i]
                            # print "second"
                            # return [990, 990, 990 ,6666]
                        else:
                            mini[q1][q2][v1][v2][0][i] = map_to(tupl4, ta, tb, <float> table[q1a+q1+adjust[0], q2a+q2+adjust[1], v1a+v1+adjust[2], v2a+v2+adjust[3], ta+adjust[4], i],
                                                            <float> table[q1a+q1+adjust[0], q2a+q2+adjust[1], v1a+v1+adjust[2], v2a+v2+adjust[3], ta+1+adjust[4], i])

                        # if mini[q1][q2][v1][v2][0][i] > 100.0:
                        #     print "!!!!!!"
                        #     print mini[q1][q2][v1][v2][0][i]
                        #     print q1a+q1, q2a+q2, v1a+v1, v2a+v2, ta, i

        # interpolate dimension 4
        for q1 in range(0, 2):
            for q2 in range(0, 2):
                for v1 in range(0, 2):
                    if -0.000001 < mini[q1][q2][v1][0][0][i] < 0.000001:
                        mini[q1][q2][v1][0][0][i] = mini[q1][q2][v1][1][0][i]
                    elif -0.000001 < mini[q1][q2][v1][1][0][i] < 0.000001:
                        mini[q1][q2][v1][0][0][i] = mini[q1][q2][v1][0][0][i]
                    else:
                        mini[q1][q2][v1][0][0][i] = map_to(tupl3, v2a, v2b, mini[q1][q2][v1][0][0][i], mini[q1][q2][v1][1][0][i])
                    # if mini[q1][q2][v1][0][0][i] > 100.0:
                    #     print "!!!!!!"
                    #     print mini[q1][q2][v1][v2][0][i]
                    #     print q1a+q1, q2a+q2, v1a+v1, v2a+v2, ta, i


        # interpolate dimension 3
        for q1 in range(0, 2):
            for q2 in range(0, 2):
                if -0.000001 < mini[q1][q2][0][0][0][i] < 0.000001:
                    mini[q1][q2][0][0][0][i] = mini[q1][q2][1][0][0][i]
                elif -0.000001 < mini[q1][q2][1][0][0][i] < 0.000001:
                    mini[q1][q2][0][0][0][i] = mini[q1][q2][0][0][0][i]
                else:
                    mini[q1][q2][0][0][0][i] = map_to(tupl2, v1a, v1b, mini[q1][q2][0][0][0][i], mini[q1][q2][1][0][0][i])
                # if mini[q1][q2][0][0][0][i] > 100.0:
                #     print "!!!!!!"
                #     print mini[q1][q2][v1][v2][0][i]
                #     print q1a+q1, q2a+q2, v1a+v1, v2a+v2, ta, i

        # interpolate dimension 2
        for q1 in range(0, 2):
            if -0.000001 < mini[q1][0][0][0][0][i] < 0.000001:
                mini[q1][0][0][0][0][i] = mini[q1][1][0][0][0][i]
            elif -0.000001 < mini[q1][1][0][0][0][i] < 0.000001:
                mini[q1][0][0][0][0][i] = mini[q1][0][0][0][0][i]
            else:
                mini[q1][0][0][0][0][i] = map_to(tupl1, q2a, q2b, mini[q1][0][0][0][0][i], mini[q1][1][0][0][0][i])
            # if mini[q1][0][0][0][0][i] > 100.0:
            #     print "!!!!!!"
            #     print mini[q1][q2][v1][v2][0][i]
            #     print q1a+q1, q2a+q2, v1a+v1, v2a+v2, ta, i


        # interpolate only remaining dimension and append to output
        if -0.000001 < mini[0][0][0][0][0][i] < 0.000001:
            result = mini[1][0][0][0][0][i]
        elif -0.000001 < mini[1][0][0][0][0][i] < 0.000001:
            result = mini[0][0][0][0][0][i]
        else:
            result = map_to(tupl0, q1a, q1b, mini[0][0][0][0][0][i], mini[1][0][0][0][0][i])

        # if len(output) > 0 and output[len(output) - 1] > 10.0:
        #     print "!!!!!!"
        #     print mini[q1][q2][v1][v2][0][i]
        #     print q1a+q1, q2a+q2, v1a+v1, v2a+v2, ta, i

        if i == 0 and -0.000001 < result < 0.000001:
            # print "not enough data to do interpolation (all zeros)"
            # print adjust[0], adjust[1], adjust[2], adjust[3], adjust[4]
            return [880.0, 880.0, 880.0, 880.0]
        else:
            output.append(result)
    return output


@cython.boundscheck(False)
@cython.wraparound(False)
cdef fill_adjust_aray(int[] adjust, int q1a, int q2a, int ta, np.ndarray table, int v1a, int v2a):
    # print "\n\n\n\n\n\n*****************************************************\nstarting new adjustment process\n\n\n\n\n\n\n"
    # print table.shape[0], table.shape[1], table.shape[2], table.shape[3], table.shape[4], table.shape[5]

    cdef int enough_data

    for i in range(5):
        adjust[i] = 0

    for i in range(<int> math.pow(3, 5)):
        count = 0
        while i > 0:
            digit = i % 3
            if digit == 2:
                digit = -1
            adjust[count] = digit
            i /= 3
            count += 1

        adjust[0] = clamp(adjust[0] + q1a, 0, table.shape[0]-2) - q1a
        adjust[1] = clamp(adjust[1] + q2a, 0, table.shape[1]-2) - q2a
        adjust[2] = clamp(adjust[2] + v1a, 0, table.shape[2]-2) - v1a
        adjust[3] = clamp(adjust[3] + v2a, 0, table.shape[3]-2) - v2a
        adjust[4] = clamp(adjust[4] + ta, 0, table.shape[4]-2) - ta
        # print "\n\n\n\n\n"
        # print table.shape[0], table.shape[1], table.shape[2], table.shape[3], table.shape[4], table.shape[5]
        enough_data = 0

        for q1 in range(0, 2):
            for q2 in range(0, 2):
                for v1 in range(0, 2):
                    for v2 in range(0, 2):
                        for t in range(0, 2):
                            if -0.0000001 < <float> table[q1a + q1 + adjust[0], q2a + q2 + adjust[1], v1a + v1 + adjust[
                                2], v2a + v2 + adjust[3], ta + t + adjust[4], 0] < 0.0000001:
                                enough_data = 1
        if enough_data == 0:
            return


cdef inline int int_max(int a, int b): return a if a >= b else b
cdef inline int int_min(int a, int b): return a if a <= b else b

cdef inline int clamp(int x, int a, int b):
    if x < a:
        return a
    if x > b:
        return b
    return x


cdef inline int fixbounds(int a, int min_val, int max_val):
    if a < min_val:
        a = min_val
    if a > max_val - 1:
        a = max_val - 1
    return a

@cython.cdivision(True)
cdef inline double map_to(double x, double in_min, double in_max, double out_min, double out_max):
    """Translates a value from one domain to another keeping proportions constant
    :type x: float
    :type in_min: float
    :type in_max: float
    :type out_min: float
    :type out_max: float
    :rtype : float
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


cdef double get_holding_torque(double q1, double q2, double w1, double w2, double l1, double l2, double m1, double m2, double g):
    """
    Returns the torque required to counteract gravity on the upper segment.
    Returns:
        float: torque required
    """
    return m2 * l2 * g * cos(q1 + q2)


cdef double get_center_of_gravity(double q1, double q2, double w1, double w2, double l1, double l2, double m1, double m2, double dt):
    """
    Returns x coridinate of the center of gravity of the acrobot.
    Args:
        tup (float,float,float,float): current state
    Returns:
        float: x coordinate of the center of gravity
    """

    m1x = l1 * m1 * cos(q1 + w1 * dt)
    m2x = m2 * (l1 * cos(q1 + w1 * dt) +
                l2 * cos(q1 + w1 * dt + q2 + w2 * dt))
    return m1x + m2x

def get_mechanical_energy(tupl, double l1, double l2, double m1, double m2, double g):
    cdef double q1, q2, w1, w2, ans
    q1 = tupl[0]
    q2 = tupl[1]
    w1 = tupl[2]
    w2 = tupl[3]
    ans = get_mechanical_energy_c(q1, q2, w1, w2, l1, l2, m1, m2, g)
    return ans


@cython.cdivision(True)
cdef double get_mechanical_energy_c(double q1_py, double q2_py, double w1_py, double w2_py, double l1_py, double l2_py, double m1_py, double m2_py, double g_py):
    """
    Returns the potential energy of the acrobot at the passed state. Throws an exception if a tuple without 4 or 5
    elements is passed in.
    :param q1_py: lower angle
    :param q2_py: upper angle
    :param w1_py: lower joint velocity
    :param w2_py: upper joint velocity
    :param l1_py: lower joint length
    :param l2_py: upper joint length
    :param m1_py: lower mass's mass
    :param m2_py: upper mass's mass
    :param g_py: gravity
    :return: potential energy of the acrobot
    :rtype : float
    """

    cdef double q1, q2, w1, w2, l1, l2, m1, m2, g, v1, v2, v1x, v1y, v2x, v2y, p1_1, p1_2, p2_1, p2_2, p2_rel_1, \
        p2_rel_2, d, theta_d, ke, pe1, pe2

    q1 = q1_py
    q2 = q2_py
    w1 = w1_py
    w2 = w2_py
    l1 = l1_py
    l2 = l2_py
    m1 = m1_py
    m2 = m2_py
    g = g_py

    # potential energy
    m1y = sin(q1) * l1
    m2y = sin(q1 + q2) * l2

    pe1 = m1y * m1 * g * -1  # -1 because gravity is also negative and PE increase as m1x increases
    pe2 = (m2y + m1y) * m2 * g * -1

    v1 = w1 * l1
    v2 = w2 * l2

    v1x = sin(q1) * v1 * -1
    v1y = cos(q1) * v1

    p1_1 = cos(q1) * l1
    p1_2 = sin(q1) * l1
    p2_rel_1 = cos(q1 + q2) * l2
    p2_rel_2 = sin(q1 + q2) * l2
    p2_1 = p1_1 + p2_rel_1
    p2_2 = p1_2 + p2_rel_2
    d = sqrt(p2_1 ** 2 + p2_2 ** 2)
    if d != 0.0:
        theta_d = acos(p2_1 / d)
        if p2_2 < 0:
            theta_d = M_PI * 2 - theta_d

        v2x = (v2 * sin(q1 + q2) + d * w1 * sin(theta_d)) * -1
        v2y = v2 * cos(q1 + q2) + d * w1 * cos(theta_d)

    else:
        v2x = v2 * sin(q1 + q2)
        v2y = v2 * cos(q1 + q2)


    ke = .5 * m1 * (v1x ** 2 + v1y ** 2) + .5 * m2 * (v2x ** 2 + v2y ** 2)
    return pe1 + pe2 + ke
