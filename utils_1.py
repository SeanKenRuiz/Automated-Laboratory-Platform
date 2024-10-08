def center_offset_calculations(index, xyxy):
    # Input: results[0].boxes.xyxy, type: tensor
    # Output: x and y offset from camera center
    ccenter_y = 240
    ccenter_x = 320

    ocenter_xy = bbox_center(xyxy[index][0], xyxy[index][1], xyxy[index][2], xyxy[index][3])

    x_offset = ocenter_xy[0] - ccenter_x
    y_offset = ocenter_xy[1] - ccenter_y

    return x_offset, y_offset, ocenter_xy

def bbox_center(l, t, r, b):
    center_x = l + ((r-l) / 2)
    center_y = t + ((b-t) / 2)

    return center_x, center_y