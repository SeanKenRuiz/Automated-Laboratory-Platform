import cv2 as cv
import numpy as np
from ultralytics import YOLO
import cv2 as cv
import numpy as np
import torch


# Load a pre-trained YOLOv10n model
model = YOLO("yolo_models/best_4.pt")


#cap = cv.VideoCapture('testTubeVid.MOV')

camera_stream = 0
cap = cv.VideoCapture(camera_stream)
# fourcc = cv.VideoWriter_fourcc(*'DIVX')
# out = cv.VideoWriter('output1.avi', cv.VideoWriter_fourcc(*'MJPG'), 20.0, (1280, 720))

#####
# Convert the image to grayscale

while True:
    # Take each frame
    _, frame = cap.read() 

    
    # Convert image to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Convert image to binary using a threshold
    _, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)

    # Define the kernel size for morphological operations
    kernel = np.ones((2,2), np.uint8)  # Adjust the kernel size as necessary

    # Noise removal using morphological opening
    opening = cv.morphologyEx(bw, cv.MORPH_OPEN, kernel, iterations=3)  # Increase iterations if necessary

    # Hole filling using morphological closing
    closing = cv.morphologyEx(bw, cv.MORPH_CLOSE, kernel, iterations=10)  # Increase iterations if necessary

    # Apply edge detection
    edges = cv.Canny(bw, 50, 150)

    inverted_image = cv.bitwise_not(edges)

    anded = cv.bitwise_and(opening, inverted_image)

    # Hole filling using morphological closing
    closing = cv.morphologyEx(inverted_image, cv.MORPH_CLOSE, kernel, iterations=10)  # Increase iterations if necessary

    # Find contours in the image
    contours, _ = cv.findContours(anded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Iterate over the contours and find rectangles
    for contour in contours:
        # Approximate the contour with a polygon
        epsilon = 0.04 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)

        # Calculate the area of each contour
        area = cv.contourArea(contour)
        
        # Ignore contours that are too small or too large
        if area < 3700 or 150000 < area:
            continue

        # If the polygon has 4 vertices, it is likely a rectangle
        if len(approx) == 4:
            # Draw the rectangle on the image
            cv.drawContours(frame, [approx], 0, (0, 255, 0), 2)

    # Show the image with the detected rectangles
    cv.imshow('Rectangles', edges)

    # Perform object detection on an image
    #results = model(frame)
    results = model.track(frame, persist=True, verbose=False, max_det=25, tracker="botsort.yaml")

    cv.imshow('Output Image', frame)

    annotated_frame = results[0].plot()

    # out.write(results[0].cpu().numpy())
    cv.imshow('frame', annotated_frame)

    cv.imshow('Original Binary Image', bw)
    cv.imshow('After Opening', opening)
    cv.imshow('After Closing', closing)
    cv.imshow('After Invert', inverted_image)
    cv.imshow('ANDED', anded)

    if cv.waitKey(1) == ord('q'):
        break

cap.release()
# out.release()
cv.destroyAllWindows()

