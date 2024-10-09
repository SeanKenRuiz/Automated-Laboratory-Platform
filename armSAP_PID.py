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
    
global tracking_index
tracking_index = 0

test_tube_diameter = 11 # in millimeters
empty_holder_diameter = 11 # in millimeters
dist_between_tubes = 19.65 # in millimeters
camera_claw_real_offset = 34.5 # in millimeters

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

        # Convert the string values to floats
        x, y, z, r = map(float, coords_list)

        # Return the coordinates in a dictionary
        return x, y, z, r
    except (IndexError, ValueError) as e:
        print(f"Error parsing return message: {e}")
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

x_controller = PDController(0.8, 0.1)
y_controller = PDController(0.8, 0.1)

# Loop to continuously get frames from the webcam
while True:
    success, frame = cap.read()  # Capture frame-by-frame

    if not success:
        print("Failed to grab frame.")
        break

    # Run YOLOv8 tracking on the frame, persisting tracks between frames
    results = model.track(frame, persist=True, verbose=False)

        # Get the boxes and track IDs
    bboxes_coord = results[0].boxes.xyxy
    if len(bboxes_coord) > 0:
        x_offset, y_offset, ocenter_x, ocenter_y, x_real_offset, y_real_offset = center_offset_calculations(tracking_index, bboxes_coord)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Convert tensor values to integers
        ocenter_x = int(ocenter_x.item())
        ocenter_y = int(ocenter_y.item())

        #print(f"PIXEL OFFSET X: {x_offset}, Y: {y_offset}")
        print(f"DOBOT X: {x_real_offset}, Y: {y_real_offset}")

        # Get current pose
        x, y, z, r = parse_get_pose(dashboard.GetPose())
        
        userparam="User=2"
        x_movement = 0
        y_movement = 0

        x_movement = x_controller.compute(x_real_offset) * -1
        y_movement = y_controller.compute(y_real_offset) * -1
        # X AXIS MOVEMENT
        # if(x_offset > 1):
        #     # If x offset of the camera is positive, decrease current x value
        #     x_movement -= 0.5 * abs(x_real_offset)   
        # elif (x_offset < -1):
        #     # If x offset of the camera is negative, increase current x value
        #     x_movement += 0.5 * abs(x_real_offset)
        # # Y AXIS MOVEMENT
        # if(y_offset > 1):
        #      # If y offset of the camera is positive, decrease current y value
        #     y_movement -= 0.5 * abs(y_real_offset)
        # elif(y_offset < -1):
        #      # If y offset of the camera is negative, increase current y value
        #     y_movement += 0.5 * abs(y_real_offset)

        # Run MovL
        move.MovL(x + x_movement, y + y_movement, z, r, userparam)
               
        #(results[0].boxes.id is not None):
            #print(f"id: {results[0].boxes.id[0]} ox: {ocenter_x}, oy: {ocenter_y}")
            #print(results[0].boxes.id)

        # Draw line from middle of the selected test tube and the center of the screen
        cv.line(annotated_frame, (ocenter_x, ocenter_y), (320, 240), (255, 0, 0), 3)

        # Display the resulting frame
        cv.imshow('YOLOv8 Webcam', annotated_frame)
    else:
        print("No bounding boxes detected.")
        cv.imshow('YOLOv8 Webcam', frame)

    # If w is pressed, allow user to change tracking index
    if cv.waitKey(1) & 0xFF == ord("w"):
        tracking_index = int(input("Enter tracking index: "))

    # Break the loop if 'q' is pressed
    if cv.waitKey(1) & 0xFF == ord("q"):
        break

# Release the webcam and close the window
cap.release()
cv.destroyAllWindows()