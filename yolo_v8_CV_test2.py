from collections import defaultdict
import cv2 as cv
import numpy as np
from ultralytics import YOLO
from ultralytics import trackers
import torch
import threading

from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
import time
from time import sleep
import numpy as np
from main import GetFeed, ClearRobotError

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

        time.sleep(0.5)
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

    print(xyxy.shape)
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
    
def perform_grab(x, y, z, r):
    # End effector offset values from target destination
    # MAKE SURE TO CALIBRATE THESE VALUES IF WEBCAM IS EVER UNMOUNTED
    # x - 28, y - 2, z = -90, r = -180
    userparam="User=0"
    if(parse_get_pose(dashboard.GetPose()) != None):
                x, y, z, r = parse_get_pose(dashboard.GetPose())
    else:
         return
    # x -= 28 # CAMERA X OFFSET VALUE FROM END EFFECTOR
    # y -= 1.6 # CAMERA Y OFFSET VALUE FROM END EFFECTOR
    x -= 29
    y -= 2.5
    r = -180
    move.MovL(x, y, z, r, userparam)
    
    finalize = int(input("Enter 0 to CANCEL grab, enter 1 to EXECUTE grab: "))
    if(finalize):
        z = -90 # TEST TUBE Y OFFSET
        move.MovL(x, y, z, r)
    
    # Close gripper
    index=1
    status=0
    dashboard.DO(index,status)

    move.MovL(x, y, z + 70, r)
    global tracking_id
    save_position(tracking_id)

def perform_place(x, y, z, r):
    # End effector offset values from target destination
    # MAKE SURE TO CALIBRATE THESE VALUES IF WEBCAM IS EVER UNMOUNTED
    # x - 28, y - 2, z = -90, r = -180
    userparam="User=0"
    if(parse_get_pose(dashboard.GetPose()) != None):
                x, y, z, r = parse_get_pose(dashboard.GetPose())
    else:
         return
    # x -= 28 # CAMERA X OFFSET VALUE FROM END EFFECTOR
    # y -= 1.6 # CAMERA Y OFFSET VALUE FROM END EFFECTOR
    x -= 29
    y -= 2.5
    r = -180
    move.MovL(x, y, z, r, userparam)
    
    finalize = int(input("Enter 0 to CANCEL release, enter 1 to EXECUTE release: "))
    if(finalize):
        z = -25 # TEST TUBE Y OFFSET
        move.MovL(x, y, z, r)
    
    # Open gripper
    index=1
    status=1
    dashboard.DO(index,status)

    move.MovL(x, y, z + 70, r)
    global tracking_id
    save_position(tracking_id)

test_tube_positions = {} 
def save_position(id):
    print("Position saved: ")
    if(parse_get_pose(dashboard.GetPose()) != None):
        x, y, z, r = parse_get_pose(dashboard.GetPose())
        test_tube_positions[id] = x, y, z, r
    else:
        return None
    
# Load a model
model = YOLO("yolo_models/yolov8_best.pt")

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

x_controller = PDController(0.9, 0.1)
y_controller = PDController(0.9, 0.1)

# Run MovL
userparam="User=0"
# Home 274, 10, 130, -180
x = 274
y = 10
z = 110
r = -180
move.MovL(x, y, z, r, userparam)
previous_x, previous_y, previous_z, previous_r = x, y, z, r

# Open gripper
index=1
status=1
dashboard.DO(index,status)

# Set action to tracking
action = 0

# Loop to continuously get frames from the webcam
while True:
    success, frame = cap.read()  # Capture frame-by-frame

    if not success:
        print("Failed to grab frame.")
        break

    # Run YOLOv8 tracking on the frame, persisting tracks between frames
    results = model.track(frame, persist=True, verbose=False)

    # Get the boxes and track IDs
    if len(results[0].boxes) > 0 and len(results[0].boxes.xyxy) > 0:
        bboxes_coord = results[0].boxes.xyxy

        if((results[0].boxes.id == tracking_id).nonzero(as_tuple=True)[0].shape[0] > 0):
            tracking_index = ((results[0].boxes.id == tracking_id).nonzero(as_tuple=True)[0].item())
        else:
            tracking_index = 0
        x_offset, y_offset, ocenter_x, ocenter_y, x_real_offset, y_real_offset = center_offset_calculations(tracking_index, bboxes_coord)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Convert tensor values to integers
        if(torch.is_tensor(ocenter_x)):
            ocenter_x = int(ocenter_x.item())
            ocenter_y = int(ocenter_y.item())

        #print(f"PIXEL OFFSET X: {x_offset}, Y: {y_offset}")
        print(f"DOBOT X: {x_real_offset}, Y: {y_real_offset}")

        if(action == 0):
            #
            #
            ## Search code start
            User=2
            Tool=0
            # Get current pose
            if(parse_get_pose(dashboard.GetPose()) != None):
                x, y, z, r = parse_get_pose(dashboard.GetPose())
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

            # Run MovL
            userparam="User=0"
            if(z > -65 and z_decrement == True):
                move.MovL(x + x_movement, y + y_movement, z - 5, r, userparam)
            else: 
                move.MovL(x + x_movement, y + y_movement, z, r, userparam)
            ## Search END
            #
            #

        elif(action == 1):
            perform_grab(x, y, z, r)
            

        # Draw line from middle of the selected test tube and the center of the screen
        cv.line(annotated_frame, (ocenter_x, ocenter_y), (320, 240), (255, 0, 0), 3)

        # Display the resulting frame
        cv.imshow('YOLOv8 Webcam', annotated_frame)
    else:
        print("No bounding boxes detected.")
        cv.imshow('YOLOv8 Webcam', frame)

    # If w is pressed, allow user to change action
    if cv.waitKey(1) & 0xFF == ord("a"):
        print("0 for tracking, 1 for grabbing, 2 for placing")
        action = int(input("Enter an action integer: "))

    # If w is pressed, allow user to change tracking index
    elif cv.waitKey(1) & 0xFF == ord("w"):
        tracking_id = int(input("Enter tracking ID: "))

    # Break the loop if 'q' is pressed
    elif cv.waitKey(1) & 0xFF == ord("q"):
        break

    #time.sleep(5)

# Release the webcam and close the window
cap.release()
cv.destroyAllWindows()