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
        
def perform_grab(x, y, z, r):
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
    move.MovL(x, y, -65, r, userparam)
    
    finalize = int(input("Enter 0 to CANCEL grab, enter 1 to EXECUTE grab: "))
    if(finalize):
        z = -90 # TEST TUBE Y OFFSET
        move.MovL(x, y, z, r)
    
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

def perform_place(x, y, z, r):
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
    move.MovL(x, y, -20, r, userparam)
    move.Sync()
    
    finalize = int(input("Enter 0 to CANCEL release, enter 1 to EXECUTE release: "))
    if(finalize):
        z = -75 # TEST TUBE Y OFFSET
        move.MovL(x, y, z, r)
    
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
    
# def bb_intersection_over_union(boxA, boxB):
# 	# determine the (x, y)-coordinates of the intersection rectangle
# 	xA = max(boxA[0], boxB[0])
# 	yA = max(boxA[1], boxB[1])
# 	xB = min(boxA[2], boxB[2])
# 	yB = min(boxA[3], boxB[3])
# 	# compute the area of intersection rectangle
# 	interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
# 	# compute the area of both the prediction and ground-truth
# 	# rectangles
# 	boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
# 	boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
# 	# compute the intersection over union by taking the intersection
# 	# area and dividing it by the sum of prediction + ground-truth
# 	# areas - the interesection area
# 	iou = interArea / float(boxAArea + boxBArea - interArea)
# 	# return the intersection over union value
# 	return iou

global test_tube_check 
def first_available_test_tube():
    first_available_test_tube_id = test_tube_check[test_tube_indices[0]]
    test_tube_check[test_tube_indices[0]] = float('inf')
    return first_available_test_tube_id

# Load a model
tray_model = YOLO("yolo_models/best_3.pt")
scale_model = YOLO("yolo_models/best_3.pt")

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

def search(x_real_offset, y_real_offset, z_decrement):
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
    #move.Sync()
    # Run MovL
    userparam="User=0"
    if(z > -65 and z_decrement == True):
        move.MovL(x + x_movement, y + y_movement, z - 5, r, userparam)
    else: 
        move.MovL(x + x_movement, y + y_movement, z, r, userparam)
    ## Search END
    #
    #

z_decrement = 0
first_tracking_index = False
jogging = False
previous_time = time.time()
first_available_test_tube_running = False

# Loop to continuously get frames from the webcam
dashboard.SpeedL(20)
dashboard.AccL(20)
dashboard.SpeedFactor(20)
action_index = -1
tray_model_running = 1

while True:
    success, frame = cap.read()  # Capture frame-by-frame

    # If not able to grab frame
    if not success:
        print("Failed to grab frame.")
        break

    if action == 4:
        # If action is to load the home frame and position, set results to home bboxes
        results = home_bbox_frame
        action = -1
    else:
        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = tray_model.track(frame, show_conf=False, persist=True, verbose=False, max_det=25, tracker="tracker_models/bytetrack.yaml")

    # GET NEXT AVAILABLE TEST TUBE
    if(first_available_test_tube_running == False):
        tracking_index = first_available_test_tube()
        test_tube_check = results[0].boxes.id.clone()

        empty_holder_indices = (results[0].boxes.cls == 0).nonzero(as_tuple=True)[0]
        test_tube_indices = (results[0].boxes.cls == 1).nonzero(as_tuple=True)[0]
        tray_indicies = (results[0].boxes.cls == 2).nonzero(as_tuple=True)[0]

        test_tube_check[tray_indicies] = float('inf')
        test_tube_check[empty_holder_indices] = float('inf')
        first_available_test_tube_running = True

    action_index += 1

    action_sequence = [0, # Search for test tube id
                       1, # Pick up test tube
                       7, # Move SAFELY to scale
                       0, # Search for empty holder id 
                       2, # Perform place
                       1, # Pick up test tube
                       6, # Move SAFELY to tray
                       2, # Perform place
                       -2] # Reset
    
    action = action_sequence[action_index]

    """
    first_available_test_tube sequence
    1. Initialize first available test tube id
        # Set first_available_test_tube_running to True
    2. Search for test tube id - CHECK
    3. Pick up test tube - CHECK

    4. STOP FEED FROM TRAY_MODEL
    5. Move SAFELY to scale
    6. START FEED FROM SCALE_MODEL

    7. Search for empty holder id
    8. Perform place
    9. Pick up again

    10. STOP FEED FROM SCALE_MODEL
    11. Move SAFELY back to tray
    12. STOP FEED FROM TRAY_MODEL

    13. Perform place in original spot
    14. Set first_available_test_tube_running to False to run function again
    """

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

    # If w is pressed, allow user to change action
    if cv.waitKey(1) & 0xFF == ord("a"):
        print("0 for tracking, 1 for grabbing, 2 for placing, 3 to set home, 4 to load home")
        action = int(input("Enter an action integer: "))
        move.Sync()
        if(action == 0):
            z_decrement = int(input("z_decrement: 0 for off, 1 for on"))
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
        search(x_real_offset, y_real_offset, z_decrement)
    elif(action == 1): # Grabbing
        perform_grab(x, y, z, r)
        action = -1 # Will be hovering over same position, but test tube -> empty holder and different id
    elif(action == 2): # Placing
        perform_place(x, y, z, r)
        action = -1
    elif(action == 3): # Set home position and frame
        set_home_position_and_frame(results)
        action = -1
    elif(action == 4): # Load home position and frame
        x, y, z, r = home_x, home_y, home_z, home_r
        previous_x, previous_y, previous_z, previous_r = x, y, z, r
        # Move to home position
        move.MovL(x, y, z, r, userparam)
        move.Sync()
        action = -1
    elif(action == 5): # Jog between positions
        tracking_index = 0
        jogging = True
        action = 0 
    elif(action == -2): # Reset action_sequence
        action_index = -1
        first_available_test_tube_running = False


        # Initialize the results with the home frame

    #time.sleep(5)

# Release the webcam and close the window
cap.release()
cv.destroyAllWindows()