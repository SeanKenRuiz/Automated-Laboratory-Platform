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
    xyxy_tensor_length = xyxy.shape[0]
    for index in range(0,xyxy_tensor_length):
        l = xyxy[index, 0]
        t = xyxy[index, 1]
        r = xyxy[index, 2]
        b = xyxy[index, 3]

        bbox_center_tensor.append(bbox_center(l,t,r,b))

    return torch.tensor(bbox_center_tensor)

def tensor_sort(tensor):
    # sorting the tensor in ascending order along the columns
    sorted_tensor, indices = torch.sort(tensor, 
                             dim=0, 
                             descending=False) 
    
    return sorted_tensor, indices

bbox = [[2.7217e+00, 2.5284e-03, 6.1554e+02, 4.6535e+02],
        [4.0691e+01, 5.2801e+01, 1.0587e+02, 1.1036e+02],
        [5.4592e+02, 2.1982e+02, 6.0418e+02, 2.7885e+02],
        [2.5062e+02, 8.1131e+01, 3.1186e+02, 1.3610e+02],
        [3.5958e+02, 8.2782e+00, 4.2179e+02, 6.4104e+01],
        [4.5741e+02, 1.0581e+02, 5.1566e+02, 1.5947e+02],
        [3.5381e+02, 9.2718e+01, 4.1340e+02, 1.4774e+02],
        [1.6291e+02, 2.5521e-03, 2.2360e+02, 3.7943e+01],
        [5.5041e+02, 1.2913e+02, 6.0796e+02, 1.8452e+02],
        [5.2011e+02, 3.3885e+02, 5.8014e+02, 4.0196e+02],
        [2.2220e+02, 2.8105e+02, 2.8420e+02, 3.4436e+02],
        [3.2895e+02, 2.9647e+02, 3.9059e+02, 3.5908e+02],
        [2.8643e+01, 2.8464e+02, 8.8061e+01, 3.4229e+02],
        [4.2318e+02, 3.2794e+02, 4.8280e+02, 3.8738e+02],
        [8.8031e+01, 3.0446e+01, 1.4240e+02, 8.1648e+01],
        [5.2915e+02, 8.5212e+01, 5.8301e+02, 1.4402e+02],
        [4.9448e+01, 1.9285e+02, 1.0616e+02, 2.5105e+02],
        [1.2995e+02, 2.9629e+02, 1.8908e+02, 3.5356e+02],
        [4.4128e+02, 7.7182e+01, 4.9043e+02, 1.2518e+02]]

# bbox = torch.tensor(bbox)
# bbox = bbox_center_tensor(bbox)
# print(bbox)

# bbox, indices = tensor_sort(bbox)
# print(bbox)
# print(indices)

A = torch.tensor([1, 2, 3, 4])
indices = torch.tensor([1, 0, 3, 2])
result = torch.tensor([0, 0, 0, 0])
print(result.scatter_(0, indices, A))