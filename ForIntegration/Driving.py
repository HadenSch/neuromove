
import numpy as np
import pandas as pd
import scipy

from scipy.spatial.transform import Rotation as Rot
from collections import deque
 
#assumptions to begin with -> all neigborhood points do not directly touch walls

def main(driving_mode, destination_mode, gyroscope_data, destination, local_input):
    destination = (0,2) #received from aleks every ???s, assuming y and x respectively
                        #Running on the assumption that the next point will be updated every ???s and not once the trainer reaches its first point

    currentpos = (0,0,0) #Defining this just in case (x,y,angle from +y) 
                        #likely to be zero since destination is respect to lidar (0,0,0) and front is always in y+

    #important Assumptions (need rotational assumtions changed):
    # Mspeed = 0.7 #in m/s
    # Maccel = 2.5 #in m/s^2
    # Mrotspeed = 15 #assumed to be in deg/s, can confirm from IMU
    # Mrotaccel = 30 #we will likely have to estimate this

    df_size = 500
    angular_speed_df = deque(maxlen=df_size)
    previous_angle = 0

    #currently for moving 1m forwards and turning 30 degrees at a time
    if driving_mode == 2:
        t_accel = 0.28 #in seconds
        t_const = 1.15 #in seconds
        t_rotaccel = 0.5 #in seconds
        t_rotconst = 1.5 #in seconds
        if local_input == 1:
            t_forward = t_accel + t_const
            t_backward = t_accel
            t_control = (t_forward, t_backward, 0, 0)
            return t_control
        if local_input == 2:
            t_left = t_rotaccel + t_rotconst
            t_right = t_rotaccel
            t_control = (0, 0, t_right, t_left)
            return t_control
        if local_input == 3:
            t_control = (0, 0, 0, 0)
            return t_control
        if local_input == 4:
            t_right = t_rotaccel + t_rotconst
            t_left = t_rotaccel
            t_control = (0, 0, t_right, t_left)
            return t_control

    while driving_mode == 1:
        



def quanternion_to_euler(gyroscope_data):
    rotation_quan = Rot.from_quat(gyroscope_data, scalar_first= True) #scalar-last order – (x, y, z, w) or scalar-first order – (w, x, y, z)
    rotation_euler = rotation_quan.as_euler('xyz', degrees=True)
    anglular_speed = -(rotation_euler[3])
    return(anglular_speed)



def angularspeed_to_angle(angular_speed_df):
    Angle = scipy.integrate.simpson(angular_speed_df, x=None, dx = 0.002)
    return Angle


def full_error(destination, currentpos):
    x_error = destination[0]-currentpos[0]
    y_error = destination[1]-currentpos[1]
    total_error = np.hypot(x_error, y_error)
    angle_error = np.arctan2(x_error,y_error)
    error = (x_error, y_error, total_error, angle_error)
    print(angle_error)
    print(error[1])
    return error


def rotate_first(error):
    #Option 1: No PID, Likely best in tight areas

    #rotates until angle_error is within 2 degrees zero 
    #then drives until the total_error is 2m of zero and stops

    #assuming 2 degree stopping distance for angular motion
    if ( -1.53588972679 < error[3] < -0.0349066):
        print('go left')
        drive = 0
    elif ( 1.53588972679 > error[3] > 0.0349066):
        print('go right')
        drive = 0
    else:
        print('stop rotation')
        drive = 1 

    #assuming 2m stopping distance for linear motion
    #assuming next point is >2 m away, wont move if next point is <2 m way
    if (drive == 1) and (error[1] > 0) and (error[2] >= 2):
        print('go forward')
    elif (drive == 1) and (error[1] < 0) and (error[2] >= 2):
        print('go backward')
    else:
        print('stop linear')

def rotate_and_move():
    # Option 2: No PID, Likely best in large areas
        
    #rotates until angle_error is within 2 degrees zero 
    #at the same time, drives until the total_error is 2m of zero and stops

    #assuming 2 degree stopping distance for angular motion
    error = full_error(destination, currentpos)
    if ( -1.53588972679 < error[3] < -0.0349066):
        print('go left')
    elif ( 1.53588972679 > error[3] > 0.0349066):
        print('go right')
    else:
        print('stop rotation') 

    #assuming 2m stopping distance for linear motion
    #assuming next point is >2 m away, wont move if next point is <2 m way
    if (error[1] > 0) and (error[2] >= 2):
        print('go forward')
    elif (error[1] < 0) and (error[2] >= 2):
        print('go backward')
    else:
        print('stop linear')


if __name__ == "__main__":
    # 0 for local driving, 1 for destination driving
    # 0 for rotate first then move, 1 for rotate and move at the same time
    main(1,0)