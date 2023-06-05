import time
import math

def VREP_getHandlesInformation(vrep_sim, clientID, elementName, referenceElementHandle):

    # elementPosition = [0, 0, 0]
    # elementOrientation = [0, 0, 0]

    # get the Handles of joint.
    return_code, elementHandle = vrep_sim.simxGetObjectHandle(clientID, elementName, vrep_sim.simx_opmode_blocking)
    if (return_code == vrep_sim.simx_return_ok):
        print('get element ',elementName,' is ok.', '\n')
        print(elementName, 'Handle is', elementHandle, '\n')
        return_code, elementPosition = vrep_sim.simxGetObjectPosition(clientID,
                                                                      elementHandle,
                                                                      referenceElementHandle,
                                                                      vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print(elementPosition, '\n')
        return_code, elementOrientation = vrep_sim.simxGetObjectOrientation(clientID,
                                                                            elementHandle,
                                                                            referenceElementHandle,
                                                                            vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print(elementOrientation, '\n')
        print('\n')
    return elementPosition, elementOrientation, elementHandle



# def VREP_velocityControl(vrep_sim, clientID,
#                          jointsHandles,
#                          desiredJointVelocity):
#     time.sleep(1)
#     for i in range(0, 7):
#         vrep_sim.simxSetJointTargetVelocity(clientID, jointsHandles[i],
#                                             desiredJointVelocity[i], vrep_sim.simx_opmode_blocking)


def VREP_positionControl(vrep_sim, clientID,
                         jointsHandles,
                         desiredJointPosition):
    for i in range(0, 7):
        if i == 3:
            vrep_sim.simxSetJointTargetPosition(clientID, jointsHandles[i],
                                                desiredJointPosition[i]-math.pi/2, vrep_sim.simx_opmode_blocking)
        elif i == 5:
            vrep_sim.simxSetJointTargetPosition(clientID, jointsHandles[i],
                                                desiredJointPosition[i]+math.pi / 2, vrep_sim.simx_opmode_blocking)
        else:
            vrep_sim.simxSetJointTargetPosition(clientID, jointsHandles[i],
                                                desiredJointPosition[i], vrep_sim.simx_opmode_blocking)