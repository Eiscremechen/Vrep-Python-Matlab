
import time
import math
import sys
import numpy as np

sys.path.append('./Function/')
sys.path.append('./Python/')

import Function.VrepFunction as VrepFunction
import Function.getT
import Function.ik_Franka_py



import Python.sim as vrep_sim


print ('--------------------------------------------------------------')
print ('import the vrep-library')




if __name__ == '__main__':

    print ('Program started')


    vrep_sim.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep_sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    if clientID!=-1:
        print ('Connected to remote API server')


    else:
        print ('Failed connecting to remote API server')


    time.sleep(1)

    joint_1_Position, joint_1_Orientation, joint_1_Handle =\
        VrepFunction.VREP_getHandlesInformation(vrep_sim,
                                                clientID,
                                                'Franka_joint1',
                                                -1)
    joint_2_Position, joint_2_Orientation, joint_2_Handle = \
        VrepFunction.VREP_getHandlesInformation(vrep_sim,
                                                clientID,
                                                'Franka_joint2',
                                                joint_1_Handle)
    joint_3_Position, joint_3_Orientation, joint_3_Handle = \
        VrepFunction.VREP_getHandlesInformation(vrep_sim,
                                                clientID,
                                                'Franka_joint3',
                                                joint_1_Handle)
    joint_4_Position, joint_4_Orientation, joint_4_Handle = \
        VrepFunction.VREP_getHandlesInformation(vrep_sim,
                                                clientID,
                                                'Franka_joint4',
                                                joint_1_Handle)
    joint_5_Position, joint_5_Orientation, joint_5_Handle = \
        VrepFunction.VREP_getHandlesInformation(vrep_sim,
                                                clientID,
                                                'Franka_joint5',
                                                joint_1_Handle)
    joint_6_Position, joint_6_Orientation, joint_6_Handle = \
        VrepFunction.VREP_getHandlesInformation(vrep_sim,
                                                clientID,
                                                'Franka_joint6',
                                                joint_1_Handle)
    joint_7_Position, joint_7_Orientation, joint_7_Handle = \
        VrepFunction.VREP_getHandlesInformation(vrep_sim,
                                                clientID,
                                                'Franka_joint7',
                                                joint_1_Handle)


    jointHandles= np.array(
        [joint_1_Handle, joint_2_Handle, joint_3_Handle,
         joint_4_Handle, joint_5_Handle, joint_6_Handle,
         joint_7_Handle])

    point_1_Position, point_1_Orientation, point_1_Handle=\
        VrepFunction.VREP_getHandlesInformation(vrep_sim,
                                                clientID,
                                                'Point1',
                                                joint_1_Handle)

    T_07 = Function.getT.jointsMatrixCalculation(joint_7_Position, joint_7_Orientation)

    T_0point1 = Function.getT.pointsMatrixCalculation(point_1_Position,point_1_Orientation)


    print (joint_7_Position,'\n', joint_7_Orientation)
    print ("T_07 is ", '\n',T_07)
    print('\n')

    print (point_1_Position, '\n', point_1_Orientation)
    print ("T_0point1 is ", '\n',T_0point1)
    print('\n')



    # l_1 = 0.330;
    # l_2 = 0.3160;
    # l_3 = 0.0880;
    # l_4 = 0.3840;
    #
    # T_01 = Function.getT.modifiedDH(0,  l_1,    0,      0           )
    # T_12 = Function.getT.modifiedDH(0,  0,      0,      -math.pi/2  )
    # T_23 = Function.getT.modifiedDH(0,  l_2,    0,      +math.pi/2  )
    # T_34 = Function.getT.modifiedDH(0,  0,      l_3,    +math.pi/2  )
    # T_45 = Function.getT.modifiedDH(0,  l_4,    -l_3,   -math.pi/2  )
    # T_56 = Function.getT.modifiedDH(0,  0,      0,      +math.pi/2  )
    # T_67 = Function.getT.modifiedDH(0,  0,      l_3,    +math.pi/2  )
    #
    # T_07 = T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56 @ T_67
    #
    T_7endeffector =  np.array([
        [1,         0,          0,      0       ],
        [0,         1,          0,      0      ],
        [0,         0,          1,      0.107  ],
        [0,         0,          0,      1      ]
    ])

    T = T_0point1 @ np.linalg.inv(T_7endeffector)

    print("T is ", '\n', T)




    # q_0, q_1, q_2, q_3, x, y, z, theta, k_x, k_y, k_z= Function.getT.rotationToQuaternions(T)
    # print ("Quaternion:", q_0, q_1, q_2, q_3, x, y, z, '\n', theta, k_x, k_y, k_z, '\n')
    # q_array = Function.ik_Franka_py.ik_Franka_qua(q_0, q_1, q_2, q_3, x, y, z)


    q_array = Function.ik_Franka_py.ik_Franka_T(T)



    print("q_array is",'\n', q_array)

    print(q_array/math.pi*180)

    # q_array =[0.195116062204703,1.01396739492712,-0.00428728610073235,
    #           0.518380330308722,0.00765347930710678,
    #           0.495595193519700,0.199582922030333]


    time.sleep(2)

    Function.VrepFunction.VREP_positionControl(vrep_sim,
                                               clientID,
                                               jointHandles,
                                               q_array)





    ''' Main control loop '''
    print('begin main control loop ...')

    # while True:
        # Motion planning

        # VREP_velocityControl(vrep_sim, clientID,
        #                      Joints_handles,
        #                      Desired_Joint_velocity)

        # VREP_PositionControl(vrep_sim, clientID,
        #                      Joints_handles,
        #                      Desired_Joint_Position)

        # vrep_sim.simxSetJointTargetVelocity(clientID, Joints_handles[0],
        #                                     Desired_Joint_velocity[0], vrep_sim.simx_opmode_blocking)


    # Now send some data to CoppeliaSim in a non-blocking fashion:
    vrep_sim.simxAddStatusbarMessage(clientID,'Over!',vrep_sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep_sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    vrep_sim.simxFinish(clientID)
    print ('Program ended')

