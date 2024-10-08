from collections import defaultdict
import cv2 as cv
import numpy as np
from ultralytics import YOLO
from ultralytics import trackers
import torch

global tracking_index
tracking_index = 0
tracking_id = 0

def center_offset_calculations(index, xyxy):
    # Input: results[0].boxes.xyxy, type: tensor
    # Output: x and y offset from camera center
    ccenter_y = 240
    ccenter_x = 320

    ocenter_xy = bbox_center(xyxy[index, 0], xyxy[index, 1], xyxy[index, 2], xyxy[index, 3])

    ocenter_x, ocenter_y = ocenter_xy

    x_offset = ocenter_x - ccenter_x
    y_offset = ocenter_y - ccenter_y

    return x_offset, y_offset, ocenter_x, ocenter_y

def bbox_center(l, t, r, b):
    center_x = l + ((r-l) / 2)
    center_y = t + ((b-t) / 2)

    return center_x, center_y

# Load a model
model = YOLO("yolo_models/yolov8_best.pt")

# Store the track history
track_history = defaultdict(lambda: [])

# Open a connection to the webcam
cap = cv.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Loop to continuously get frames from the webcam
while True:
    tracking_id = results[0].boxes.id
    success, frame = cap.read()  # Capture frame-by-frame

    if not success:
        print("Failed to grab frame.")
        break

    # Run YOLOv8 tracking on the frame, persisting tracks between frames
    results = model.track(frame, persist=True, verbose=False)

    # Get the boxes and track IDs
    bboxes_coord = results[0].boxes.xyxy
    if len(bboxes_coord) > 0:
        x_offset, y_offset, ocenter_x, ocenter_y = center_offset_calculations(tracking_index, bboxes_coord)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Convert tensor values to integers
        ocenter_x = int(ocenter_x.item())
        ocenter_y = int(ocenter_y.item())

        if(results[0].boxes.id is not None):
            print(f"id: {results[0].boxes.id[0]} ox: {ocenter_x}, oy: {ocenter_y}")

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