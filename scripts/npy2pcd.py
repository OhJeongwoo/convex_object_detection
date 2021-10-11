import numpy as np
import struct
import sys
import open3d as o3d
import os
import time
import rospkg


def npy_to_bin(npyFileName, binFileName):
    data = np.load(npyFileName)
    data.astype('float32').tofile(binFileName)

def bin_to_pcd(binFileName, pcdFileName):
    size_float = 4
    list_pcd = []
    with open(binFileName, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, -y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    o3d.io.write_point_cloud(pcdFileName, pcd)
    return 

data_name = "demo1"
N = 10
data_path = rospkg.RosPack().get_path("convex_object_detection") + "/data/" + data_name + "/"
npy_path = data_path + "lidar/"
bin_path = data_path + "bin/"
pcd_path = data_path + "pcd/"


for seq in range(N):
    print(seq)
    npy_file = npy_path + str(seq).zfill(6) + ".npy"
    bin_file = bin_path + str(seq).zfill(6) + ".bin"
    pcd_file = pcd_path + str(seq).zfill(6) + ".pcd"
    # npy_to_bin(npy_file, bin_file)
    time.sleep(0.1)
    bin_to_pcd(bin_file, pcd_file)