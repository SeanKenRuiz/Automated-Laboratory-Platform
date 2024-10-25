# from ultralytics import YOLO

# # Load YOLOv10n model from scratch
# model = YOLO("yolov10n")

# # Train the model
# model.train(data=, epochs = 100, imgsz)

from ultralytics import YOLO
import cv2 as cv
import numpy as np
import torch

def bbox_center(l, t, r, b):
    center_x = l + ((r-l) / 2)
    center_y = t + ((b-t) / 2)

    return center_x, center_y

def bbox_center_tensor(xyxy):
    """
    Input: tensor of bbox xyxy coordinates
    Output: tensor of bbox center coordinates
    """
    bbox_center_tensor = []
    xyxy_tensor_length = xyxy.shape[0].item()
    for index in range(0,xyxy_tensor_length):
        l = xyxy[index, 0]
        t = xyxy[index, 1]
        r = xyxy[index, 2]
        b = xyxy[index, 3]

        bbox_center_tensor.append([l,t,r,b])

    return torch.tensor(bbox_center_tensor)

# Load a pre-trained YOLOv10n model
model = YOLO("yolo_models/best_4.pt")

#cap = cv.VideoCapture('testTubeVid.MOV')

camera_stream = 1
cap = cv.VideoCapture(camera_stream)
# fourcc = cv.VideoWriter_fourcc(*'DIVX')
# out = cv.VideoWriter('output1.avi', cv.VideoWriter_fourcc(*'MJPG'), 20.0, (1280, 720))

while True:
    # Take each frame
    _, frame = cap.read()

    # Perform object detection on an image
    #results = model(frame)
    results = model.track(frame, persist=True, verbose=False, max_det=25, tracker="botsort.yaml")
    print(results[0].boxes.cls)

    annotated_frame = results[0].plot()

    # out.write(results[0].cpu().numpy())
    cv.imshow('frame', annotated_frame)

    if cv.waitKey(1) == ord('q'):
        break

cap.release()
# out.release()
cv.destroyAllWindows()