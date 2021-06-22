import os
import math
import pandas as pd

TIME_CONVERSION_CONST_ = 10 ** 9

def calculate_rosbot_velocities(x_new, y_new, rpy_new, x_prev, y_prev, rpy_prev, dt):
    """
    Calculates the linear and angular velocity of the rosbot
    """
    # get yaw from [roll, pitch, yaw]
    
    yaw_new = rpy_new[2]   
    yaw_prev = rpy_prev[2] 
    
    d_yaw = yaw_new - yaw_prev
    d_yaw = (d_yaw + math.pi) % (2 * math.pi) - math.pi
    w = d_yaw / dt

    vx = (x_new - x_prev) / dt
    vy = (y_new - y_prev) / dt
    v = math.sqrt(vx**2 + vy**2)

    alpha = math.atan2(vy,vx)
    v = v * math.cos(alpha - yaw_new)
    return v, w

def calculate_next_kinematic_model_state(control_df):
    """
    """
    pass

def convert_ros2_time_to_float(time_tuple):
    """
    Converts time from tuple (ROS2) format to floating point number
    """
    return float(time_tuple[0] + time_tuple[1]/TIME_CONVERSION_CONST_)

def parse_csv_file(file_path):
    """
    Args:
        :file_path: (str)
    Return:
        :data: (pandas.DataFrame)
    """
    return pd.read_csv(
        file_path,
        sep=' ',
    )
    
def parse_logger_output_data(folder_path):
    """
    :Args:
        :folder_path: (str) - path to the folder with logger data
    :Return:
        :data: (list of pd.DataFrame) - output data files
    """
    data = 5 * [pd.DataFrame()]

    required_files = [
        'rosbot_state.csv', 
        'kinematic_model_state.csv', 
        'nn_model_state.csv', 
        'control.csv', 
        'time.csv'
    ]

    for i, file in zip(range(5), required_files):
        data[i] = parse_csv_file(folder_path+'/'+file)
    
    return data



