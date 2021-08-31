import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

DT_MAX = 0.1                  # maximum time delta
DT_MIN = 0.03                # minimal time delta
Tmax = 50                     # control_generator running time
UPDATE_RATE = np.random.uniform(1/DT_MIN, 1/DT_MAX) # sample rosbot update_rate


import subprocess

subprocess.Popen("ros2 launch rosbot_gazebo bringup.launch.py world:=/home/user/empty gui:=false rosbot_update_rate:={}".format(int(UPDATE_RATE)), shell=True)
# os.popen("ros2 launch rosbot_description rosbot_sim.launch.py world:=/home/user/empty gui:=true rosbot_update_rate:={}".format(UPDATE_RATE))


output_path = "/home/user/ros2_ws/src/logger/output_data"
rclpy.init()
data_collector = Node("data_collector")
cli = data_collector.create_client(Empty, 'shutdown_logger')
req = Empty.Request()

def ResetPose():
    """
    Teleports the rosbot to the origin
    """
    res = os.popen("ros2 service call /reset_simulation std_srvs/srv/Empty")


def run_control_gen(
    output_folder,
    Tmax,
    v_max,
    w_max,
    a_lin,
    a_ang,
    period_lin,
    period_ang,
    UPDATE_RATE
    ):

    """
    Launch the control_generator,
    and wait for the end of the trajectory 
    """

    run_ = os.popen(
        f"ros2 launch rosbot_controller collect_data.launch.py output_path:={output_folder} pub_rate:={int(UPDATE_RATE)} Tmax:={Tmax} v_max:={v_max} w_max:={w_max} a_lin:={a_lin} a_ang:={a_ang} period_lin:={period_lin} period_ang:={period_ang} v_min:={-v_max} w_min:={-w_max}"
        )

    time.sleep(5)
    while "control_gen" in data_collector.get_node_names():
        time.sleep(1)
        
    time.sleep(2)
    future = cli.call_async(req)
    ResetPose() # teleport the rosbot to the origin




time.sleep(10)

# Sample trajectory parameters
v_max = round(np.random.uniform(low=0.0, high=1.5), 2)    # max linear velocity
w_max = round(np.random.uniform(low=0.0, high=2.5), 2)    # max angular velocity
a_lin = round(np.random.uniform(low=-1.0, high=1.0), 4)   # linear acceleration
a_ang = round(np.random.uniform(low=-1.0, high=1.0), 4)   # angular acceleration
period_lin = 25 # np.random.randint(5, int(Tmax/2))       # linear velocity changing period
period_ang = 25 # np.random.randint(5, int(Tmax/2))       # angular velocity changing period

first_output_folder = "rosbot_2d_dt_{}_Tmax={}_v_max={}_w_max={}_a_lin={}_a_ang={}_per_lin={}_per_ang={}/".format(
    round(1/UPDATE_RATE, 3),
    Tmax,
    v_max,
    w_max,
    a_lin,
    a_ang,
    period_lin,
    period_ang
)
output_folder = os.path.join(output_path, first_output_folder)
run_control_gen(output_folder, Tmax, v_max, w_max, a_lin, a_ang, period_lin, period_ang, UPDATE_RATE)

# Composing the second trajectory folder name
a_ang = - a_ang
second_output_folder = "rosbot_2d_dt_{}_Tmax={}_v_max={}_w_max={}_a_lin={}_a_ang={}_per_lin={}_per_ang={}/".format(
    round(1/UPDATE_RATE, 3),
    Tmax,
    v_max,
    w_max,
    a_lin,
    a_ang,
    period_lin,
    period_ang
)
output_folder = os.path.join(output_path, second_output_folder)
# Collect the second trajectory
run_control_gen(output_folder, Tmax, v_max, w_max, a_lin, a_ang, period_lin, period_ang, UPDATE_RATE)
