from collections import defaultdict
import cv2 as cv
import numpy as np
from ultralytics import YOLO
from ultralytics import trackers
import torch
import threading

from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from dobot_main import GetFeed, ClearRobotError
import time
from time import sleep
import numpy as np

class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.previous_error = 0
        self.previous_time = time.time()

    def compute(self, error):
        current_time = time.time()
        delta_time = current_time - self.previous_time
        delta_error = error - self.previous_error

        if delta_time <= 0.0:
            return 0

        p_term = self.kp * error
        d_term = self.kd * (delta_error / delta_time)

        self.previous_error = error
        self.previous_time = current_time

        #time.sleep(0.50)
        return p_term + d_term

# Connecting to robot arm
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
    
global tracking_id
tracking_id = 1

test_tube_diameter = 11 # in millimeters
empty_holder_diameter = 11 # in millimeters
dist_between_tubes = 19.65 # in millimeters

def center_offset_calculations(index, xyxy):
    """
    Input: results[0].boxes.xyxy, type: tensor
    Output: 
    x offset from camera center, in pixels
    y offset from camera center, in pixels
    object center x value, in pixels
    object center y value, in pixels

    x offset from camera center, in millimeters
    y offset from camera center, in millimeters
    """

    ccenter_y = 240
    ccenter_x = 320

    l = xyxy[index, 0]
    t = xyxy[index, 1]
    r = xyxy[index, 2]
    b = xyxy[index, 3]

    # get b
    ocenter_xy = bbox_center(l, t, r, b)

    ocenter_x, ocenter_y = ocenter_xy

    x_offset = ocenter_x - ccenter_x
    y_offset = ocenter_y - ccenter_y

    x_real_offset, y_real_offset = convert_pixels_to_millimeters(x_offset, y_offset, l, t, r, b)

    # IMPORTANT: X AND Y ARE FLIPPED B/C ARM HAS OPPOSITE AXES FROM CAMERA
    return y_offset, x_offset, ocenter_x, ocenter_y, y_real_offset, x_real_offset

def bbox_center(l, t, r, b):
    center_x = l + ((r-l) / 2)
    center_y = t + ((b-t) / 2)

    return center_x, center_y

def bbox_center_tensor(xyxy):
    """
    Input: tensor of bbox xyxy coordinates
    Output: tensor of bbox center coordinates
    """
    return

def convert_pixels_to_millimeters(pixel_x, pixel_y, l, t, r, b):
    x_real = pixel_x * test_tube_diameter / (r - l) # pixels * millimeters / horizontal bbox pixels
    y_real = pixel_y * test_tube_diameter / (b - t) # pixels * millimeters / vertical bbox pixels
    return x_real, y_real

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
        
def perform_grab(z2, is_camera_centered):
    
    """
    Have the MG400 robot arm perform grab sequence 

    Parameters:
    int: the z value the object is at
    boolean: is the camera centered over the object?

    Returns:
    None
    """
    # End effector offset values from target destination
    # MAKE SURE TO CALIBRATE THESE VALUES IF WEBCAM IS EVER UNMOUNTED
    # Additionally, if this position may differ depending on how far J1 has to rotate from its x/y axes

    # x - 28, y - 2, z = -90, r = -180

    # Open gripper
    index=1
    status=1
    dashboard.DO(index,status)
    
    userparam="User=0"
    
    if(parse_get_pose(dashboard.GetPose()) != None):
        x, y, z, r = parse_get_pose(dashboard.GetPose())
        previous_x, previous_y, previous_z, previous_r = x, y, z, r
    else:
        return
    # x -= 28 # CAMERA X OFFSET VALUE FROM END EFFECTOR
    # y -= 1.6 # CAMERA Y OFFSET VALUE FROM END EFFECTOR
    dashboard.SpeedFactor(100)
    if(is_camera_centered and using_tray_model):
        x -= 25
        y += 1.7
        r = -185
        move.MovL(x, y, z2+25, r, userparam)
    if(is_camera_centered and not using_tray_model):
        x -= 25.97
        y += 1.01
        r = -185
        move.MovL(x, y, z2+25, r, userparam)
    
    finalize = int(input("Enter 0 to CANCEL grab, enter 1 to EXECUTE grab: "))
    if(finalize):
        move.MovL(x, y, z2, r)
    
        # Close gripper
        index=1
        status=0
        dashboard.DO(index,status)

        move.MovL(x, y, z + 70, r, userparam)
        # Save position hovering over tube
        global tracking_id
        save_position(tracking_id)
        print(test_tube_positions)

    # Clear the area
    move.MovL(x, y, 110, r, userparam)
    # Return to camera-centered_hovering position
    move.MovL(previous_x, previous_y, previous_z, previous_r, userparam)

    move.Sync()
    dashboard.SpeedFactor(20)

def perform_place(z2):
    # End effector offset values from target destination
    # MAKE SURE TO CALIBRATE THESE VALUES IF WEBCAM IS EVER UNMOUNTED
    # x - 28, y - 2, z = -90, r = -180
    userparam="User=0"
    if(parse_get_pose(dashboard.GetPose()) != None):
        x, y, z, r = parse_get_pose(dashboard.GetPose())
        previous_x, previous_y, previous_z, previous_r = x, y, z, r
    else:
        return
    # x -= 28 # CAMERA X OFFSET VALUE FROM END EFFECTOR
    # y -= 1.6 # CAMERA Y OFFSET VALUE FROM END EFFECTOR
    dashboard.SpeedFactor(100)
    x -= 25
    y += 1.7
    r = -185
    move.MovL(x, y, z2 + 55, r, userparam)
    move.Sync()
    
    finalize = int(input("Enter 0 to CANCEL release, enter 1 to EXECUTE release: "))
    if(finalize):
        # z2 = -75 for tray, 50 for scale
        move.MovL(x, y, z2, r)
    
        # Open gripper
        index=1
        status=1
        dashboard.DO(index,status)

    # Clear the area
    move.MovL(x, y, 110, r)
    # Return to camera-centered hovering position
    move.MovL(previous_x, previous_y, previous_z, previous_r, userparam)

    move.Sync()
    dashboard.SpeedFactor(20)

    global tracking_id
    save_position(tracking_id)
    print(test_tube_positions)

test_tube_positions = {} 
def save_position(id):
    print("Position saved: ")
    if(parse_get_pose(dashboard.GetPose()) != None):
        x, y, z, r = parse_get_pose(dashboard.GetPose())
        test_tube_positions[id] = x, y, z, r
    else:
        return None
    
global home_x, home_y, home_z
home_r = -186
global home_bbox_frame

def set_home_position_and_frame(bounding_box_tensor_tuple):
    global home_bbox_frame
    global home_x, home_y, home_z
    home_bbox_frame = bounding_box_tensor_tuple
    if(parse_get_pose(dashboard.GetPose()) != None):
        home_x, home_y, home_z, _ = parse_get_pose(dashboard.GetPose())
    else:
        print("setting home position failed... attempting again")
        set_home_position_and_frame()
   
# Load a model
tray_model = YOLO("yolo_models/best_3.pt")

#scale_model = YOLO("yolo_models/best_3.pt")

# Store the track history
track_history = defaultdict(lambda: [])

# Open a connection to the webcam
cap = cv.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Connect to robot arm
dashboard, move, feed = connect_robot()

# Initialize robot arm
load=0.400 # around the total load the end effector will be handling, in kilograms
dashboard.EnableRobot(load)

x_controller = PDController(1, 0.05)
y_controller = PDController(1, 0.05)

# Run MovL
userparam="User=0"
# Home 274, 10, 130, -180
global x, y, z, r
x = 274
y = 10
z = 110
r = -186
move.MovL(x, y, z, r, userparam)
previous_x, previous_y, previous_z, previous_r = x, y, z, r

# Open gripper
index=1
status=1
dashboard.DO(index,status)

# Set action to tracking
action = 0

dashboard.User(0)
dashboard.Tool(0)
move.Sync()

def search(x_real_offset, y_real_offset):
    #
    #
    ## Search code start

    # Get current pose
    #move.Sync()
    if(parse_get_pose(dashboard.GetPose()) != None):
        global x, y, z, r
        x, y, z, r = parse_get_pose(dashboard.GetPose())
        global previous_x, previous_y, previous_z, previous_r
        previous_x, previous_y, previous_z, previous_r = x, y, z, r
    else:
        # Keep previous pose
        x = previous_x
        y = previous_y
        z = previous_z
        r = previous_r

    
    x_movement = 0
    y_movement = 0

    x_movement = x_controller.compute(x_real_offset) * -1
    y_movement = y_controller.compute(y_real_offset) * -1
    #move.Sync() THIS SYNC CAUSES FRAMES TO PROCESS SEQUENTIALLY  

    # If in tray area OR if movement is within the scale x-axis min and maximum values
    if using_tray_model or ((not using_tray_model) and (x + x_movement > SCALE_MIN_X and x + x_movement < SCALE_MAX_X)):
        # Run MovL
        userparam="User=0"
        move.MovL(x + x_movement, y + y_movement, z, r, userparam)
    else:
        print("Search: Movement out of bounds")
    ## Search END
    #
    #

z_decrement = 0
first_tracking_index = False
jogging = False
previous_time = time.time()
using_tray_model = True
holding_tube = False
camera_unlocked = True
global action_buffer_previous_time

# Define where on the range of the x-axis when hovering within scale
SCALE_MIN_X = 25
SCALE_MAX_X = 90

dashboard.SpeedL(20)
dashboard.AccL(20)
dashboard.SpeedFactor(20)

# Loop to continuously get frames from the webcam
# To end loop, click on Python image frame and press 'q'
while True:
    success, frame = cap.read()  # Capture frame-by-frame

    # If not able to grab frame
    if not success:
        print("Failed to grab frame.")
        break

    # Run YOLOv8 tracking on the frame, persisting tracks between frames
    results = tray_model.track(frame, show_conf=False, persist=True, verbose=False, max_det=25, tracker="tracker_models/bytetrack.yaml")

    # IF does NOT contain any IDs AND the specified ID index
    if(((results[0].boxes.id != None) and ((results[0].boxes.id == tracking_id).nonzero(as_tuple=True)[0].shape[0] > 0)) == False):
        annotated_frame = results[0].plot(conf=False)
        cv.imshow('YOLOv8 Webcam', annotated_frame)
        x_real_offset, y_real_offset = 0, 0
        print("Defaulting to first_tracking_index available...")
        print("Change id to existing index")
        first_tracking_index = True

    # If results has bounding boxes to go along with those bounding boxes
    elif((len(results[0].boxes) > 0) and (len(results[0].boxes.xyxy) > 0)):
        current_time = time.time() - previous_time

        # If jogging between positions enabled
        if(jogging == True and current_time > 5):
            save_position(tracking_index)
            tracking_index += 1
            previous_time = time.time()
        
        # Draw line to the center of the tracking_id's bbox
        bboxes_coord = results[0].boxes.xyxy    # Assign bounding box coordinates to the variable bboxes_coord
        if(first_tracking_index == False and jogging == False):
            tracking_index = ((results[0].boxes.id == tracking_id).nonzero(as_tuple=True)[0].item()) # Assign index of the tracking_id to the variable tracking_index

        # Calculate pixel offset from the user-specified tracking_id
        x_offset, y_offset, ocenter_x, ocenter_y, x_real_offset, y_real_offset = center_offset_calculations(tracking_index, bboxes_coord)

        # Visualize the results on the frame and turn of confidence
        annotated_frame = results[0].plot(conf=False)

        # Convert tensor values to integers
        if(torch.is_tensor(ocenter_x)):
            ocenter_x = int(ocenter_x.item())
            ocenter_y = int(ocenter_y.item())

        #print(f"PIXEL OFFSET X: {x_offset}, Y: {y_offset}")
        print(f"DOBOT X: {x_real_offset}, Y: {y_real_offset}")

        # Draw line from middle of the selected test tube and the center of the screen
        cv.line(annotated_frame, (ocenter_x, ocenter_y), (320, 240), (255, 0, 0), 3)

        # Display the resulting frame
        cv.imshow('YOLOv8 Webcam', annotated_frame)
    else:
        print("No bounding boxes detected.")
        print("Defaulting to first_tracking_index available...")
        first_tracking_index = True
        cv.imshow('YOLOv8 Webcam', frame)

    # Get time since action = -1 was set
    if(action == -1):
        action_buffer_time = time.time() - action_buffer_previous_time

    # If w is pressed, allow user to change action
    if cv.waitKey(1) & 0xFF == ord("a") or (action == -1 and action_buffer_time > 5):
        print("0 for tracking, 1 for grabbing, 2 for placing, 3 to set home, 4 to load home")
        print("5 for jogging, 6 move SCALE to TRAY, 7 move TRAY to SCALE, 8 PLACE in SCALE, 9 PICK from SCALE ")        
        action = int(input("Enter an action integer: "))
        move.Sync()
        # Action code that activates only once
        if(action == -1):
            action_buffer_previous_time = time.time()
        elif(action == 0):
            tracking_id = int(input("Enter tracking ID: "))
            first_tracking_index = False

    # If w is pressed, allow user to change tracking index
    elif cv.waitKey(1) & 0xFF == ord("w"):
        tracking_id = int(input("Enter tracking ID: "))
        first_tracking_index = False
    # Break the loop if 'q' is pressed
    elif cv.waitKey(1) & 0xFF == ord("q"):
        break

    # Action states
    if((action == 0) and (parse_robot_mode(dashboard.RobotMode()) != 7)): # Searching
        if(using_tray_model):
            results = tray_model.track(frame, show_conf=False, persist=True, verbose=False, max_det=25, tracker="tracker_models/bytetrack.yaml")
        else:
            results = tray_model.track(frame, show_conf=False, persist=True, verbose=False, max_det=10, tracker="tracker_models/bytetrack.yaml")
        annotated_frame = results[0].plot(conf=False)
        cv.imshow('YOLOv8 Webcam', annotated_frame)
        search(x_real_offset, y_real_offset)

    elif(action == 1): # Grabbing
        if(using_tray_model):
            perform_grab(-90, True)
        elif(using_tray_model == False):
            perform_grab(40, True)
        action = -1 # Will be hovering over same position, but test tube -> empty holder and different id
        action_buffer_previous_time = time.time()
        holding_tube = True

    elif(action == 2): # Placing
        if(using_tray_model):
            perform_place(-75)
        elif(using_tray_model == False):
            perform_place(70)
        action = -1
        action_buffer_previous_time = time.time()
        holding_tube = False

    elif(action == 3): # Set home position and frame
        set_home_position_and_frame(results)
        action = -1
        action_buffer_previous_time = time.time()

    elif(action == 4): # Load home position and frame
        x, y, z, r = home_x, home_y, home_z, home_r
        previous_x, previous_y, previous_z, previous_r = x, y, z, r
        # Move to home position
        move.MovL(x, y, z, r, userparam)
        move.Sync()
        using_tray_model = True
        action = -1
        action_buffer_previous_time = time.time()

    elif(action == 5): # Jog between positions
        tracking_index = 0
        jogging = True
        action = 0

    elif(action == 6): # Move from SCALE to TRAY
        dashboard.SpeedFactor(100)
        # MAKE SURE TO MOVE BACK TO PRE-SCALE POSITION BEFORE ALL MOVES FROM SCALE
        # Scale to pre-scale position
        x, y, z, r = 68, 210, 170, -186
        move.MovL(x, y, z, r, userparam)
        move.Sync()

        # Go back to tray
        # Alignment point (point on the arc)
        x = 127
        y = 183
        z = 146
        r = -186
        # Target coordinates
        x2 = 274
        y2 = 10
        z2 = 110
        r2 = -186
        move.Arc(x, y, z, r, x2, y2, z2, r2)  
        move.Sync()
        action = -1
        using_tray_model = True
        dashboard.SpeedFactor(20)
        camera_unlocked = True
        action_buffer_previous_time = time.time()

    elif(action == 7): # Move from TRAY to SCALE
        dashboard.SpeedFactor(100)
        # Go from tray to pre-scale position
        x = 127
        y = 183
        z = 146
        r = -186
        # Target coordinates (pre-scale)
        x2, y2, z2, r2 = 68, 210, 170, -186
        move.Arc(x, y, z, r, x2, y2, z2, r2)  
        move.Sync()

        # Scale hover position
        x, y, z, r = 61, 305, 160, -186
        move.MovL(x, y, z, r, userparam)
        move.Sync()
        action = -1
        using_tray_model = False
        dashboard.SpeedFactor(20)
        camera_unlocked = True
    elif(action == 8): # DROP INTO SCALE
        dashboard.SpeedFactor(100)
        # Scale hover position
        x, y, z, r = 25.94, 358.59, 69.58, -186
        move.MovL(x, y, z, r, userparam)
        move.Sync()
        action = -1
        action_buffer_previous_time = time.time()

        # Open gripper
        index=1
        status=1
        dashboard.DO(index,status)

        using_tray_model = True
        dashboard.SpeedFactor(20)
    elif(action == 9): # Pick FROM SCALE
        # Open gripper
        index=1
        status=1
        dashboard.DO(index,status)

        dashboard.SpeedFactor(100)
        # Scale hover position
        x, y, z, r = 25.94, 358.59, 42, -186
        move.MovL(x, y, z, r, userparam)
        move.Sync()

        # Close gripper
        index=1
        status=0
        dashboard.DO(index,status)

        # Scale hover position
        x, y, z, r = 25.94, 358.59, 42, -186
        move.MovL(x, y, z, r, userparam)
        move.Sync()

        action = -1
        action_buffer_previous_time = time.time()

        using_tray_model = True
        move.Sync()

        x, y, z, r = 50, 342.65, 154.89, -186
        move.MovL(x, y, z, r, userparam)
        move.Sync()
        dashboard.SpeedFactor(20)

    #time.sleep(5)

# Release the webcam and close the window
cap.release()
cv.destroyAllWindows()