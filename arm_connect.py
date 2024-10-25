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

    # Home 274, 10, 130, -180
    x = 274
    y = 10
    z = 70
    r = -180
    userparam="User=0"
    move.MovL(x, y, z, r,userparam)
    sleep(5)
    dashboard.RobotMode()
    print(parse_robot_mode(dashboard.RobotMode()) == 5)
    dashboard.GetPose()
    