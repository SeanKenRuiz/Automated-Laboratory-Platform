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

def tray_tilted_right(results):
    #bboxes = results[0].boxes
    ids = results[0].boxes.id
    xyxy = results[0].boxes.xyxy

    # Get indices of empty holders and test tubes
    #empty_holder_indices = (results[0].boxes.cls == 0).nonzero(as_tuple=True)[0]
    #test_tube_indices = (results[0].boxes.cls == 1).nonzero(as_tuple=True)[0]
    tray_indicies = (results[0].boxes.cls == 2).nonzero(as_tuple=True)[0]

    print(results[0].boxes.xyxy.shape[0])
    # Apply the mask to the tensor
    min_masked_xyxy = xyxy.clone()
    min_masked_xyxy[tray_indicies] = float('inf')

    max_masked_xyxy = xyxy.clone()
    max_masked_xyxy[tray_indicies] = float('-inf')

    
    # print(min_masked_xyxy)
    # print(max_masked_xyxy)

    # Find the minimum value in each column (dim=0 specifies columns)
    min_ltrb_values, min_indices = torch.min(min_masked_xyxy, dim=0)

    # Find the maximum value in each column (dim=0 specifies columns)
    max_ltrb_values, max_indices = torch.max(max_masked_xyxy, dim=0)

    # print(min_ltrb_values) 
    # print(min_indices)

    # Get left max, top max, right max, bottom max
    left_min_index = min_indices[0].item()
    top_max_index = max_indices[1].item()
    right_max_index = max_indices[2].item()
    bottom_min_index = min_indices[3].item()
    print(f"left_max_index: {left_min_index}, top_max_index: {top_max_index}, right_max_index: {right_max_index}, bottom_max_index: {bottom_min_index}")

    # Tilt left = l_x < t_x < b_x < r_x
    # tilt right = l_x < b_x < t_x < r_x
    print(xyxy[left_min_index, 0])
    print(xyxy[top_max_index, 0])
    print(xyxy[left_min_index, 0] < xyxy[top_max_index, 0])
    if(xyxy[left_min_index, 0] < xyxy[top_max_index, 0]):
        print('hi')
    # If tilted left
    if((xyxy[left_min_index, 0] < xyxy[top_max_index, 0]) and (xyxy[top_max_index, 0] < xyxy[bottom_min_index, 0]) and (xyxy[bottom_min_index, 0] < xyxy[right_max_index, 0])):
        #bbox_slope = (xyxy[top_max_index, 1] - xyxy[left_min_index, 1]) / (xyxy[top_max_index, 0] - xyxy[left_min_index, 0])
        bbox_slope = -(xyxy[right_max_index, 1] - xyxy[top_max_index, 1]) / (xyxy[right_max_index, 0] - xyxy[top_max_index, 0])
        return False, bbox_slope
    # If tilted right
    elif((xyxy[left_min_index, 0] < xyxy[bottom_min_index, 0]) and (xyxy[bottom_min_index, 0] < xyxy[top_max_index, 0]) and (xyxy[top_max_index, 0] < xyxy[right_max_index, 0])):
        #bbox_slope = (xyxy[right_max_index, 1] - xyxy[top_max_index, 1]) / (xyxy[right_max_index, 0] - xyxy[top_max_index, 0])
        bbox_slope = -(xyxy[top_max_index, 1] - xyxy[left_min_index, 1]) / (xyxy[top_max_index, 0] - xyxy[left_min_index, 0])
        return True, bbox_slope
    else:
        return 
    
def custom_sort(xyxy):
    sorted_tray = xyxy.clone()
    sorted_tray[0:xyxy.shape[0]-1] = float('inf')

    left_min_index = 0 ##### MAKE MINS AND MAXES INTO FUNCTION AND CALL IT HERE
    top_max_index = 0

    row_first_node = True

    dist_lower_bound = float('-inf')
    dist_upper_bound = float('inf')

    current_top = xyxy[top_max_index, 1]
    current_right = xyxy[top_max_index, 2]
    tray_tilted_right(results)

    # Take first node's right side, find closest left side (abs(current_right-next_left))
    if(tray_tilted_right):
        # Set current top to the top_max_index's top
        current_top = xyxy[top_max_index, 1]

        # Subtract all bbox values from xyxy to find distance
        distance_from_current_t = xyxy - current_top
        
        #xyxy[top_max_index] = float('inf')
        min_dist, min_dist_indices = torch.min(distance_from_current_t, dim=0)
        min_dist_lr = min_dist[1]
        min_dist_lr_index = min_dist_indices[1]

        # Save current node
        sorted_tray.append[xyxy[top_max_index]]
        # Delete from available xyxy bboxes
        xyxy = xyxy[top_max_index] = float('inf')

        # If distance between current_right and next_left within expected range,
        if((min_dist_lr >= dist_upper_bound) and (min_dist_lr <= dist_lower_bound)):
            # Move onto the next node of the next_left
            current_right = xyxy[min_dist_lr_index, 2]

        else: # If abnormal distance, node is an end node
            # Clear out current node
            # Change to next first row node available
            current_right = 
        if(row_first_node == True):
            dist_lower_bound = min_dist_lr * 0.75
            dist_upper_bound = min_dist_lr * 1.25
        
            


    # connect
    # save distance
    # Close off node
    # Move to next node
    # Go to next node's right side, find closest left side
    # If next node's distance != ~distance +- 0.25distance, then go to next "first node" according to tilt
    # end once all nodes gone

# Load a pre-trained YOLOv10n model
model = YOLO("yolo_models/best_3.pt")

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
    print(f"Tray tilted right:  {tray_tilted_right(results)}")

    annotated_frame = results[0].plot()

    # out.write(results[0].cpu().numpy())
    cv.imshow('frame', annotated_frame)

    if cv.waitKey(1) == ord('q'):
        break

cap.release()
# out.release()
cv.destroyAllWindows()
