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

    # # Home 274, 10, 130, -180
    # x = 274
    # y = 10
    # z = 70
    # r = -180
    # userparam="User=0"
    # move.MovL(x, y, z, r,userparam)

    # sleep(5)

    # dashboard.RobotMode()
    # print(parse_robot_mode(dashboard.RobotMode()) == 5)

    global x, y, z, r

    dashboard.GetPose()
    #x, y, z, r = parse_get_pose(dashboard.GetPose())

    dashboard.SpeedL(20)
    dashboard.AccL(20)
    dashboard.SpeedFactor(20)
    # Run Arc
    userparam="User=0"

    # Go back to tray
    # Alignment point (point on the arc)
    x = 224
    y = 200
    z = 110
    r = -186
    # Target coordinates
    x2 = 274
    y2 = 10
    z2 = 110
    r2 = -186
    move.Arc(x, y, z, r, x2, y2, z2, r2)  
    move.Sync()

    # Go from tray to pre-scale position
    x = 224
    y = 200
    z = 110
    r = -186
    # Target coordinates
    x2, y2, z2, r2 = 65, 210, 170, -186
    move.Arc(x, y, z, r, x2, y2, z2, r2)  
    move.Sync()

    # Scale hover position
    x, y, z, r = 51, 400, 110, -186
    move.MovL(x, y, z, r, userparam)
    move.Sync()

    # # Home 274, 10, 130, -180
    # x = 274
    # y = 10
    # z = 110
    # r = -186
    # move.MovL(x, y, z, r, userparam)

    # # Pre-scale position
    # x, y, z, r = 65, 210, 170, -186
    # move.MovL(x, y, z, r, userparam)
    # move.Sync()

    # # Scale hover position
    # x, y, z, r = 51, 400, 110, -186
    # move.MovL(x, y, z, r, userparam)
    # move.Sync()

    # Pre-scale position
    # 65, 210, 170, -186

    # Scale hover position
    # 51, 400, 110, -186

    # Scale hover position
    # 33.805526, 409.0818, 94.840012, -135.524536

    # 29.13489, 411.028036, 55.411091, -126.675667

