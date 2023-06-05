import math

import numpy as np
from math import cos, sin



def modifiedDH(theta,d,a,alpha):
    T_z_theta = np.array([

        [cos(theta),    -sin(theta),    0,          0],
        [sin(theta),    cos(theta),     0,          0],
        [0,             0,              1,          0],
        [0,             0,              0,          1]
    ])
    T_z_d = np.array([
        [1,             0,              0,          0],
        [0,             1,              0,          0],
        [0,             0,              1,          d],
        [0,             0,              0,          1]
    ])
    T_x_a =np.array([
        [1,             0,              0,          a],
        [0,             1,              0,          0],
        [0,             0,              1,          0],
        [0,             0,              0,          1]
    ])

    T_x_alpha = np.array([
        [1,             0,              0,          0],
        [0,             cos(alpha),     -sin(alpha),0],
        [0,             sin(alpha),     cos(alpha), 0],
        [0,             0,              0,          1]
    ])

    T = T_x_alpha @ T_x_a @ T_z_d @ T_z_theta

    return T
def jointsMatrixCalculation(position,orientation):

    x = position [0]
    y = position [1]
    z = position [2]

    c_alpha = cos(orientation[0])
    s_alpha = sin(orientation[0])

    c_belta = cos(orientation[1])
    s_belta = sin(orientation[1])

    c_gammar = cos(orientation[2])
    s_gammar = sin(orientation[2])

    R_z_gammar = np.array([
        [c_gammar,      -s_gammar,      0],
        [s_gammar,      c_gammar,       0],
        [0,             0,              1]
    ])

    R_y_belta = np.array([
        [c_belta,   0,          s_belta ],
        [0,         1,          0       ],
        [-s_belta,  0,          c_belta ]
    ])

    R_x_alpha = np.array([
        [1,         0,          0           ],
        [0,         c_alpha,    -s_alpha    ],
        [0,         s_alpha,    c_alpha     ]
    ])

    Rotation = R_z_gammar @ R_y_belta @ R_x_alpha

    Translation = np.array([[x],
                            [y],
                            [z]])


    T = np.concatenate((Rotation, Translation), axis=1)
    T = np.vstack((T, [0, 0, 0, 1]))

    return T


def pointsMatrixCalculation (position,orientation):
    T = jointsMatrixCalculation(position, orientation)


    return T


def rotationToQuaternions(T):

    theta = math.acos((T[0][0] + T[1][1] + T[2][2] - 1)/2)

    k_x = (T[2][1] - T[1][2]) / (2 * math.sin(theta))
    k_y = (T[0][2] - T[2][0]) / (2 * math.sin(theta))
    k_z = (T[1][0] - T[0][1]) / (2 * math.sin(theta))

    q_0 = math.cos(theta/2)
    q_1 = k_x * math.sin(theta / 2)
    q_2 = k_y * math.sin(theta / 2)
    q_3 = k_z * math.sin(theta / 2)


    # q_0 = (T[0][0] + T[1][1] + T[2][2] + 1)**0.5
    # q_1 = (T[1][2] - T[2][1]) / (4*q_0)
    # q_2 = (T[2][0] - T[0][2]) / (4*q_0)
    # q_3 = (T[0][1] - T[1][2]) / (4 * q_0)
    x = T[0][3]
    y = T[1][3]
    z = T[2][3]
    return q_0, q_1, q_2, q_3, x, y, z, theta, k_x, k_y, k_z


