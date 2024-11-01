import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from time import sleep
import numpy as np
from dobot_main import GetFeed, ClearRobotError

##  Connecting to robot arm
#
PARAMS = 0
def connect_robot():
    try:
        ip = "192.168.1.6"
        dashboard_p = 29999
        move_p = 30003
        feed_p = 30004
        print("Establishing connection...")
        dashboard = DobotApiDashboard(ip, dashboard_p)
        move = DobotApiMove(ip, move_p)
        feed = DobotApi(ip, feed_p)
        print(">.< Connection successful >!<")
        return dashboard, move, feed
    except Exception as e:
        print(":( Connection failed :(")
        raise e
    
def parse_robot_mode(return_msg):
    """
    Parse the return message from RobotMode() to get the current status of the robot.

    Parameters:
    return_msg (str): The return message string from RobotMode()

    Returns:
    int: the robot mode value.
    """
    try:
        # Extract the substring containing the coordinates
        robot_mode_value = return_msg.split(",")[1].strip("{}")

        # convert robot_mode_value from string to int
        return int(robot_mode_value)   

    except (IndexError, ValueError) as e:
        print(f"Error parsing return message: {e}")
        return None
    
def parse_get_pose(return_msg):
    """
    Parse the return message from GetPose() to extract the Cartesian coordinates.

    Parameters:
    return_msg (str): The return message string from GetPose()

    Returns:
    tuple: a tuple containing x, y, z, and r values.
    """
    try:
        # Extract the substring containing the coordinates
        coords_list = return_msg.split(",")[1:5]  
        coords_list[0] = coords_list[0].strip("{")
        print(coords_list) 

        # Convert the string values to floats
        x, y, z, r = map(float, coords_list)

        # Return the coordinates in a dictionary
        return x, y, z, r
    except (IndexError, ValueError) as e:
        print(f"Error parsing return message: {e}")
        return None

class PlaneBoundary:
    def __init__(self, pt1, pt2, pt3, pt4 ):
        self.pt1 = pt1
        self.pt2 = pt2
        self.pt3 = pt3
        self.pt4 = pt4

# 40, 305.12, z, -96
# 25.45, 353.04, z, -96
# 10, 350, z, -96
SCALE_MIN_X = 25
SCALE_MAX_X = 40

## Running script
#
if __name__ == '__main__':

    dashboard, move, feed = connect_robot()

    """
        if PARAMS  Conditional compilation directive has parameters
            0:  Instruction without parameters
            1:  Instruction with parameters
    """
    # Initialize robot arm
    load=0.400 # around the total load the end effector will be handling, in kilograms
    dashboard.EnableRobot(load)
    
    # Controlling digital output port
    index=1
    status=0
    dashboard.DO(index,status)  

    