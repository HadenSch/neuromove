#%%
import numpy as np

#%%


#assumptions to begin with -> all neigborhood points do not directly touch walls

destination = (0,2) #received from aleks every ???s, assuming y and x respectively
                     #Running on the assumption that the next point will be updated every ???s and not once the trainer reaches its first point

currentpos = (0,0,0) #Defining this just in case (x,y,angle from +y) 
                     #likely to be zero since destination is respect to lidar (0,0,0) and front is always in y+

Mspeed = 0.7 #in m/s
Maccel = 2.5 #in m/s^2
Mrotspeed = 0.7 #received from IMU every 0.02s, assumed to be in deg/s
Mrotaccel = 2.5 #we will likely have to estimate this

#%%
def full_error(destination, currentpos):
    x_error = destination[0]-currentpos[0]
    y_error = destination[1]-currentpos[1]
    total_error = np.hypot(x_error, y_error)
    angle_error = np.arctan2(x_error,y_error)
    error = (x_error, y_error, total_error, angle_error)
    print(angle_error)
    print(error[1])
    return error


#%%
class rotate_first:
    #Option 1: No PID, Likely best in tight areas

    #rotates until angle_error is within 2 degrees zero 
    #then drives until the total_error is 2m of zero and stops

    #assuming 2 degree stopping distance for angular motion
    error = full_error(destination, currentpos)
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

#%%
class rotate_and_move:
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

#%%

#Option 3: PID, likely best in tight areas, no need to assume stopping distance

#issue 1: if the point changes the derivative part will be crazy huge
#issue 2: if the point changes the integral part will be huge

#takes gyrosope data for angular velocity
#assumed angular acceleration
import matplotlib.pyplot as plt
from scipy.integrate import odeint

time = 0
integral = 0
time_prev = -1e-6
e_prev = 0

def PID(Kp, Ki, Kd, error):
    global time, integral, time_prev, e_prev

    # Value of offset - when the error is equal to this it is zero
    offset = 0
    
    # PID calculations
    e = error
        
    P = Kp*e
    integral = integral + Ki*e*(time - time_prev)
    D = Kd*(e - e_prev)/(time - time_prev)

    # calculate manipulated variable - MV 
    MV = offset + P + integral + D
    
    # update stored data for next iteration
    e_prev = e
    time_prev = time
    return MV

def system(rot accel, error):
    Mspeed = 0.7 #in m/s
    Maccel = 2.5 #in m/s^2
    rotspeed = 0.7 #received from IMU every 0.02s, assumed to be in deg/s
    rotaccel = 2.5


#%%
#Option 4: Model Predictive and Stanley based controller (MPS), likely best in large areas, no need to assume stopping distance

#takes gyrosope data for angular velocity
#assumed angular acceleration
