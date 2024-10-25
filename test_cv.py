# from ultralytics import YOLO

# # Load YOLOv10n model from scratch
# model = YOLO("yolov10n")

# # Train the model
# model.train(data=, epochs = 100, imgsz)

from ultralytics import YOLO
import cv2 as cv
import numpy as np
import torch
from math import atan2, cos, sin, sqrt, pi
import time

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

def get_and_display_object_orientations(frame, tray_bbox):
    object_orientations = []
    # Convert image to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Convert image to binary
    _, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    
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
        center = (int(rect[0][0]), int(rect[0][1])) 
        width = int(rect[1][0])
        height = int(rect[1][1])
        angle = int(rect[2])
        
        contour_box = xywh_to_xyxy(center[0], center[1], width, height)

        if width < height:
            angle = 90 - angle
        else:
            angle = -angle
                
        label = "  Rotation Angle: " + str(angle) + " degrees"

        print(tray_bbox)
        print(contour_box)

        tray_bbox = (torch.squeeze(tray_bbox))
        contour_box = list(contour_box)

        print(tray_bbox)
        print(contour_box)
        print('hi')
        tray_bbox = np.array(tray_bbox).astype(np.int32)
        contour_box = np.array(contour_box).astype(np.int32)
        print(tray_bbox)
        print(contour_box)

        cv.rectangle(frame, (tray_bbox[0], tray_bbox[1]), (tray_bbox[2], tray_bbox[3]), (0,0,255), 3)
        cv.rectangle(frame, (contour_box[0], contour_box[1]), (contour_box[2], contour_box[3]), (255,0,0))
        
        print(intersection_over_union(tray_bbox, contour_box))

        if(intersection_over_union(tray_bbox, contour_box) > 0.5):
            textbox = cv.rectangle(frame, (center[0]-35, center[1]-25), (center[0] + 295, center[1] + 10), (255,255,255), -1)
            cv.putText(frame, label, (center[0]-50, center[1]), 
                cv.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 1, cv.LINE_AA)
            cv.drawContours(frame,[box],0,(0,0,255),2)

            object_orientations.append((center, width, height, angle))

        tray_bbox = torch.from_numpy(tray_bbox)
    if(len(object_orientations) > 0):
        return object_orientations[0]
    else:
        return 0, 0, 0, 0

def xywh_to_xyxy(x, y, w, h):
    x1, y1 = x-w/2, y-h/2
    x2, y2 = x+w/2, y+h/2
    return x1, y1, x2, y2

def intersection_over_union(bbox, cbox):
    # coordinates of the area of intersection.
    ix1 = np.maximum(bbox[0], cbox[0])
    iy1 = np.maximum(bbox[1], cbox[1])
    ix2 = np.minimum(bbox[2], cbox[2])
    iy2 = np.minimum(bbox[3], cbox[3])
     
    # Intersection height and width.
    i_height = np.maximum(iy2 - iy1 + 1, np.array(0.))
    i_width = np.maximum(ix2 - ix1 + 1, np.array(0.))
     
    area_of_intersection = i_height * i_width
     
    # Ground Truth dimensions.
    gt_height = bbox[3] - bbox[1] + 1
    gt_width = bbox[2] - bbox[0] + 1
     
    # Prediction dimensions.
    pd_height = cbox[3] - cbox[1] + 1
    pd_width = cbox[2] - cbox[0] + 1
     
    area_of_union = gt_height * gt_width + pd_height * pd_width - area_of_intersection
     
    iou = area_of_intersection / area_of_union
     
    return iou

# Load a pre-trained YOLOv10n model
model = YOLO("yolo_models/best_4.pt")

#cap = cv.VideoCapture('testTubeVid.MOV')

camera_stream = 0
cap = cv.VideoCapture(camera_stream)
# fourcc = cv.VideoWriter_fourcc(*'DIVX')
# out = cv.VideoWriter('output1.avi', cv.VideoWriter_fourcc(*'MJPG'), 20.0, (1280, 720))

while True:
    # Take each frame
    _, frame = cap.read() 

    # Perform object detection on an image
    #results = model(frame)
    results = model.track(frame, persist=True, verbose=False, max_det=25, tracker="botsort.yaml")

    annotated_frame = results[0].plot()

    if((results[0].boxes.cls != None) and ((results[0].boxes.cls == 2).nonzero(as_tuple=True)[0].shape[0] > 0)):
        center, w, h, a = get_and_display_object_orientations(frame, results[0].boxes.xyxy[(results[0].boxes.cls == 2).nonzero(as_tuple=True)[0]])

    if(center != 0 and w != 0 and h != 0 and a != 0):
    # out.write(results[0].cpu().numpy())
        cv.imshow('frame', frame)
    else:
        cv.imshow('frame', frame)

    if cv.waitKey(1) == ord('q'):
        break

cap.release()
# out.release()
cv.destroyAllWindows()