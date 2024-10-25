# This programs calculates the orientation of an object.
# The input is an image, and the output is an annotated image
# with the angle of otientation for each object (0 to 180 degrees)
 
from ultralytics import YOLO
import cv2 as cv
import numpy as np
import torch
from math import atan2, cos, sin, sqrt, pi
import time

# Access the webcam
cap = cv.VideoCapture(0)

# Load a pre-trained YOLOv10n model
model = YOLO("yolo_models/best_4.pt")

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    # Take each frame
    _, frame = cap.read()

    results = model.track(frame, persist=True, verbose=False, max_det=25, tracker="botsort.yaml")

    bbox = results[0].boxes.xyxy[(results[0].boxes.cls == 2).nonzero(as_tuple=True)[0]]
    bbox = (torch.squeeze(bbox))
    bbox = np.array(bbox).astype(np.int32)

    print(bbox)
    # Create a mask or sub-image of the tray based on the bounding box
    #tray_region = frame[bbox[1]:bbox[3], bbox[0]:bbox[2]]
    
    # Convert image to grayscale
    #gray = cv.cvtColor(tray_region, cv.COLOR_BGR2GRAY)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Convert image to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Convert image to binary using a threshold
    _, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)

    # Define the kernel size for morphological operations
    kernel = np.ones((10,10), np.uint8)  # Adjust the kernel size as necessary

    # Noise removal using morphological opening
    opening = cv.morphologyEx(bw, cv.MORPH_OPEN, kernel, iterations=3)  # Increase iterations if necessary

    # Hole filling using morphological closing
    closing = cv.morphologyEx(bw, cv.MORPH_CLOSE, kernel, iterations=10)  # Increase iterations if necessary

    # Optionally, you can perform these steps to further clean the image:
    # Remove any small noise left out by opening
    sure_bg = cv.dilate(closing, kernel, iterations=3)

    # Finding sure foreground area using distance transform
    dist_transform = cv.distanceTransform(closing, cv.DIST_L2, 5)
    _, sure_fg = cv.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)

    # Finding unknown region
    sure_fg = np.uint8(sure_fg)
    unknown = cv.subtract(sure_bg, sure_fg)
    
    # Find all the contours in the thresholded image
    contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    
    for i, c in enumerate(contours):
    
        # Calculate the area of each contour
        area = cv.contourArea(c)
        
        # Ignore contours that are too small or too large
        if area < 3700 or 100000 < area:
            continue
        
        # cv.minAreaRect returns:
        # (center(x, y), (width, height), angle of rotation) = cv2.minAreaRect(c)
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect)
        box = np.intp(box)
        
        # Retrieve the key parameters of the rotated bounding box
        center = (int(rect[0][0]),int(rect[0][1])) 
        width = int(rect[1][0])
        height = int(rect[1][1])
        angle = int(rect[2])
        
            
        if width < height:
            angle = 90 - angle
        else:
            angle = -angle
                
        label = "  Rotation Angle: " + str(angle) + " degrees"
        textbox = cv.rectangle(frame, (center[0]-35, center[1]-25), 
            (center[0] + 295, center[1] + 10), (255,255,255), -1)
        cv.putText(frame, label, (center[0]-50, center[1]), 
            cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 1, cv.LINE_AA)
        cv.drawContours(frame,[box],0,(0,0,255),2)
    
    cv.imshow('Output Image', frame)
    cv.imshow('bw', sure_fg)
    # You can visualize each step to see the effect
    cv.imshow('Original Binary Image', bw)
    cv.imshow('After Opening', opening)
    cv.imshow('After Closing', closing)
    cv.imshow('Sure Background', sure_bg)
    cv.imshow('Sure Foreground', sure_fg)
    cv.imshow('Unknown Regions', unknown)
    if cv.waitKey(1) == ord('q'):
        break

cap.release()
cv.destroyAllWindows()