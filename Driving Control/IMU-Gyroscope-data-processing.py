
import numpy as np
import pandas as pd
import scipy
import time

from scipy.spatial.transform import Rotation as Rot
from collections import deque


def main(gyroscope_data):
    df_size = 100
    angular_speed_df = deque(maxlen=df_size)
    previous_angle = 0
    angle = 0
    t_control = 0

    if time.time() > t_control
        angular_speed = quanternion_to_euler(gyroscope_data)
        angular_speed_df.append(angular_speed)
        t_control = time.time() + 0.002
        
    if len(angular_speed_df) == df_size:
        previous_angle = angularspeed_to_angle(angular_speed_df)
        angle = previous_angle + angle
        return previous_angle, angle


def quanternion_to_euler(gyroscope_data):
    rotation_quan = Rot.from_quat(gyroscope_data, scalar_first= True) #scalar-last order – (x, y, z, w) or scalar-first order – (w, x, y, z)
    rotation_euler = rotation_quan.as_euler('xyz', degrees=True)
    anglular_speed = -(rotation_euler[3])
    return(anglular_speed)


def angularspeed_to_angle(angular_speed_df):
    Angle = scipy.integrate.simpson(angular_speed_df, x=None, dx = 0.002)
    return Angle


if __name__ == "__main__":
    # 1 for right side, 2 for left side
    # 0 for no output video, 1 for output video
    main()