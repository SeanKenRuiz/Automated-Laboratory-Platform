from collections import defaultdict
import cv2 as cv
import numpy as np
from ultralytics import YOLO
from ultralytics import trackers
import torch

def center_offset_calculations(index, xyxy):
    # Input: results[0].boxes.xyxy, type: tensor
    # Output: x and y offset from camera center
    ccenter_y = 240
    ccenter_x = 320

    ocenter_xy = bbox_center(xyxy[index][0], xyxy[index][1], xyxy[index][2], xyxy[index][3])

    ocenter_x, ocenter_y = ocenter_xy

    x_offset = ocenter_x - ccenter_x
    y_offset = ocenter_y - ccenter_y

    return x_offset, y_offset, ocenter_x, ocenter_y

def bbox_center(l, t, r, b):
    center_x = l + ((r-l) / 2)
    center_y = t + ((b-t) / 2)

    return center_x, center_y

TRACKING_ID = 0

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

    # # Get the boxes and track IDs
    # boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
    bboxes_coord = results[0].boxes.xyxy
    x_offset, y_offset, ocenter_x, ocenter_y = center_offset_calculations(0, bboxes_coord)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    print(bboxes_coord)
    print(bboxes_coord[0])
    print(ocenter_x, ocenter_y)
    print(type(ocenter_x))

    cv.line(annotated_frame, (ocenter_x.numpy(), ocenter_y.numpy()), (320, 240), (255, 0, 0), 5)
    
    # Display the resulting frame
    cv.imshow('YOLOv8 Webcam', annotated_frame)
    
    # # Plot the tracks
    # for box, track_id in zip(boxes, track_ids):
    #     x, y, w, h = box
    #     track = track_history[track_id]
    #     track.append((float(x), float(y))) # x, y center point
    #     if len(track) > 30: # retain 90 tracks for 90 frames
    #         track.pop(0)

        # Draw the tracking lines
        # points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
        # cv.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)
    
    # Break the loop if 'q' is pressed
    if cv.waitKey(1) & 0xFF == ord("q"):
        break

# Release the webcam and close the window
cap.release()
cv.destroyAllWindows()