from collections import defaultdict
import cv2 as cv
import numpy as np
from ultralytics import YOLO
from ultralytics import trackers
import torch

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

    # get bbox center from the left, top, right, bottom coordinates (x, y, x, y)
    ocenter_xy = bbox_center(l, t, r, b)

    ocenter_x, ocenter_y = ocenter_xy

    x_offset = ocenter_x - ccenter_x
    y_offset = ocenter_y - ccenter_y

    x_real_offset, y_real_offset = convert_pixels_to_millimeters(x_offset, y_offset, l, t, r, b)

    return x_offset, y_offset, ocenter_x, ocenter_y, x_real_offset, y_real_offset

def bbox_center(l, t, r, b):
    center_x = l + ((r-l) / 2)
    center_y = t + ((b-t) / 2)

    return center_x, center_y

def convert_pixels_to_millimeters(pixel_x, pixel_y, l, t, r, b):
    x_real = pixel_x * test_tube_diameter / (r - l) # pixels * millimeters / horizontal bbox pixels
    y_real = pixel_y * test_tube_diameter / (b - t) # pixels * millimeters / vertical bbox pixels
    return x_real, y_real

# Load a model
model = YOLO("yolo_models/yolov8_best.pt")

# Store the track history
track_history = defaultdict(lambda: [])

# Open a connection to the webcam
cap = cv.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

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
    print(results[0])
    if len(bboxes_coord) > 0:
        x_offset, y_offset, ocenter_x, ocenter_y, x_real_offset, y_real_offset = center_offset_calculations(tracking_index, bboxes_coord)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Convert tensor values to integers
        ocenter_x = int(ocenter_x.item())
        ocenter_y = int(ocenter_y.item())

        scale_factor_x = 10/6
        scale_factor_y = 3/5

        x_real_offset *= scale_factor_x
        y_real_offset *= scale_factor_y

        print(f"DOBOT X: {y_real_offset}, Y: {x_real_offset}") # FLIP X AND Y BECAUSE ARM REFERENCE DIFFERENT FROM CAMERA REFERENCE

        #(results[0].boxes.id is not None):
            #print(f"id: {results[0].boxes.id[0]} ox: {ocenter_x}, oy: {ocenter_y}")
            #print(results[0].boxes.id)

        # Draw line from middle of the selected test tube and the center of the screen
        cv.line(annotated_frame, (ocenter_x, ocenter_y), (320, 240), (255, 0, 0), 3)

        # Display the resulting frame
        cv.imshow('YOLOv8 Webcam 2', annotated_frame)
        cv.imshow('YOLOv8 Webcam 1', frame)
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