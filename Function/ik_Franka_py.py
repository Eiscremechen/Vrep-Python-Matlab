import matlab
import matlab.engine
import time
import numpy as np
from numpy import double

def unitVectorization (T):

    for i in range(0,3):
        sum = np.power (T[0][i], 2) + np.power (T[1][i], 2) + np.power (T[2][i], 2)
        T[0][i] = T[0][i] / sum
        T[1][i] = T[1][i] / sum
        T[2][i] = T[2][i] / sum
        print (i, " Columns sum is ",sum )
    return T

def ik_Franka_Qua (q_0, q_1, q_2, q_3, x, y, z):
    eng = matlab.engine.start_matlab()

    x = matlab.double(x)
    y = matlab.double(y)
    z = matlab.double(z)
    q_0 = matlab.double(q_0)
    q_1 = matlab.double(q_1)
    q_2 = matlab.double(q_2)
    q_3 = matlab.double(q_3)


    q_i = eng.iK_Franka(q_0, q_1, q_2, q_3, x, y, z)


    q_array = np.array([np.round(q_i[0][0], 6),
                        np.round(q_i[0][1], 6),
                        np.round(q_i[0][2], 6),
                        np.round(q_i[0][3], 6),
                        np.round(q_i[0][4], 6),
                        np.round(q_i[0][5], 6),
                        np.round(q_i[0][6], 6)])

    # print()
    eng.quit()

    return q_array

def ik_Franka_T (T):
    eng = matlab.engine.start_matlab()

    T=matlab.double (T)


    q_i = eng.iK_Franka_T(T)


    q_array = np.array([np.round(q_i[0][0], 6),
                        np.round(q_i[0][1], 6),
                        np.round(q_i[0][2], 6),
                        np.round(q_i[0][3], 6),
                        np.round(q_i[0][4], 6),
                        np.round(q_i[0][5], 6),
                        np.round(q_i[0][6], 6)])

    # print()
    eng.quit()

    return q_array