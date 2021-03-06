import rospy
import rospkg
import json
import numpy as np
import math
import cv2
import time


def get_pixel(x, y):
    px = int((x - maxX) / (minX - maxX) * bev_height)
    py = int((y - maxY) / (minY - maxY) * bev_width)

    return px, py

def draw_object(image, objects, type):
    for i in range(len(objects)):
        cx = objects[i][1]
        cy = objects[i][2]
        l = objects[i][5]
        w = objects[i][6]
        theta = objects[i][4]
        box = []
        box.append(get_pixel(cx + l/2 * math.cos(theta) - w/2 * math.sin(theta), cy + l/2 * math.sin(theta) + w/2 * math.cos(theta)))
        box.append(get_pixel(cx - l/2 * math.cos(theta) - w/2 * math.sin(theta), cy - l/2 * math.sin(theta) + w/2 * math.cos(theta)))
        box.append(get_pixel(cx - l/2 * math.cos(theta) + w/2 * math.sin(theta), cy - l/2 * math.sin(theta) - w/2 * math.cos(theta)))
        box.append(get_pixel(cx + l/2 * math.cos(theta) + w/2 * math.sin(theta), cy + l/2 * math.sin(theta) - w/2 * math.cos(theta)))
        box.append(get_pixel(cx, cy))
        if type == 0:
            cv2.line(image, (box[0][1], box[0][0]), (box[1][1], box[1][0]), (0, 0, 255), 2)
            cv2.line(image, (box[1][1], box[1][0]), (box[2][1], box[2][0]), (0, 0, 255), 2)
            cv2.line(image, (box[2][1], box[2][0]), (box[3][1], box[3][0]), (0, 0, 255), 2)
            cv2.line(image, (box[3][1], box[3][0]), (box[0][1], box[0][0]), (0, 255, 255), 2)
        else :
            cv2.line(image, (box[0][1], box[0][0]), (box[1][1], box[1][0]), (255, 0, 0), 2)
            cv2.line(image, (box[1][1], box[1][0]), (box[2][1], box[2][0]), (255, 0, 0), 2)
            cv2.line(image, (box[2][1], box[2][0]), (box[3][1], box[3][0]), (255, 0, 0), 2)
            cv2.line(image, (box[3][1], box[3][0]), (box[0][1], box[0][0]), (255, 255, 0), 2)

data_name = "traj3"
n_data = 260
offset = 181

LIDAR_Z = 1.7
minX = 0.0
maxX = 40.0
minY = -20.0
maxY = 20.0
minZ = 0.5 - LIDAR_Z
maxZ = 2.0 - LIDAR_Z

resolution = 1.0
bev_height = 40
bev_width = 40
cx = (minX + maxX) / 2.0
cy = (minY + maxY) / 2.0

maxlogDensity = math.log(20)

data_path = rospkg.RosPack().get_path("convex_object_detection") + "/data/" + data_name + "/"
obj_path = data_path + "object/"
lbl_path = data_path + "label/"
bin_path = data_path + "bin/"
out_path = data_path + "out/"

for seq in range(offset, offset + n_data):
    obj_file = obj_path + str(seq).zfill(6) + ".txt"
    lbl_file = lbl_path + str(seq).zfill(6) + ".txt"
    # lbl_file = lbl_path + "Object" + str(seq) + ".txt"
    bin_file = bin_path + str(seq).zfill(6) + ".bin"
    out_file = out_path + str(seq).zfill(6) + ".png"

    objects = []
    with open(obj_file, "r") as f:
        for line in f: 
            box = line.split()
            obj = []
            for i in range(0,8):
                obj.append(float(box[i]))
            objects.append(obj)

    labels = []
    with open(lbl_file, "r") as f:
        for line in f: 
            box = line.split()
            obj = []
            for i in range(0,8):
                obj.append(float(box[i]))
            labels.append(obj)


    # build bev map
    pcs = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)
    x = pcs[:,0]
    y = pcs[:,1]
    z = pcs[:,2]
    intensity = pcs[:,3]
    
    indices = []
    for i in range(len(pcs)):
        if x[i] > minX and x[i] < maxX and y[i] > minY and y[i] < maxY and z[i] > minZ and z[i] < maxZ:
            indices.append(i)
    pcs = pcs[indices,:]
    x = x[indices]
    y = y[indices]
    z = z[indices]
    intensity = intensity[indices]
    n_points = len(intensity)
    
    resolution = 0.1
    bev_height = 400
    bev_width = 400
    maxlogDensity = math.log(20)

    intensity_layer = np.zeros([bev_height, bev_width], dtype=np.float)
    density_layer = np.zeros([bev_height, bev_width], dtype=np.float)
    height_layer = np.zeros([bev_height, bev_width], dtype=np.float)
    for i in range(n_points):
        px, py = get_pixel(x[i], y[i])
        if px < 0 or px >= bev_height or py < 0 or py >= bev_width:
            continue
        intensity_layer[px][py] = max(intensity_layer[px][py], intensity[i])
        density_layer[px][py] += 1
        height_layer[px][py] = max(height_layer[px][py], (z[i]-minZ)/ (maxZ-minZ))
    for i in range(bev_height):
        for j in range(bev_width):
            density_layer[px][py] = min(1.0, math.log(1 + density_layer[px][py]) / maxlogDensity)
    intensity_layer = intensity_layer * 255.0
    density_layer = density_layer * 255.0
    height_layer = height_layer * 255.0
    intensity_layer = np.expand_dims(intensity_layer.astype('uint8'), axis = 0)
    density_layer = np.expand_dims(density_layer.astype('uint8'), axis = 0)
    height_layer = np.expand_dims(height_layer.astype('uint8'), axis = 0)
    local_map = np.transpose(np.vstack((intensity_layer, density_layer, height_layer)), (1,2,0))
    local_map = cv2.resize(local_map, (bev_height, bev_width))

    draw_object(local_map, objects, 0)
    draw_object(local_map, labels, 1)

    cv2.imwrite(out_file, local_map)

    if seq%10 == 0:
        print(seq)
