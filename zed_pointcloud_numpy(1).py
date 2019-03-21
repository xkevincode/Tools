# -*- coding: utf-8 -*-
# @Time     : 19-3-13 下午4:39
# @File     : zed_PointCloud_numpy.py
# @Author   : Kevin X

import pyzed.sl as sl
import math
import numpy as np
import sys
from open3d import *
import ctypes
import datetime

# since the zed pointcloud color is stored in float32 BGRX format,
# we need to split them and assign the values to corresponding values
def float2decimal(num):
    # this function transforms float32 to string type and
    # then split it into 4 Bytes of a float32 data
    binNum = bin(ctypes.c_uint.from_buffer(ctypes.c_float(num)).value)[2:]
    mantissa = "1" + binNum[-23:]
    mantInt = int(mantissa, 2) / 2 ** 23
    base = int(binNum[-31:-23], 2) - 127
    sign = 1 - 2 * ("1" == binNum[-32:-31].rjust(1, "0"))

    '''Uncomment it if you want to check'''
    #print("bits: " + binNum.rjust(32, "0"))
    #print("sig (bin): " + mantissa.rjust(24))
    # print("sig (float): " + str(mantInt))
    # print("base:" + str(base))
    # print("sign:" + str(sign))
    # print("recreate:" + str(sign * mantInt * (2 ** base)))

    # split the data into 4 Bytes and then transform it to decimal
    B = str(binNum.rjust(32, "1"))[0:8]
    G = str(binNum.rjust(32, "0"))[8:16]
    R = str(binNum.rjust(32, "0"))[16:24]
    alPha = str(binNum.rjust(32, "0"))[24:32]

    # changing from binary int to decimal
    B_r = bin2decimal(int(B))
    G_r = bin2decimal(int(G))
    R_r = bin2decimal(int(R))
    alPha_r = bin2decimal(int(alPha))

    return B_r, G_r, R_r, alPha_r


def bin2decimal(binary):
    # binary element to decimal
    sum = 0
    counter = 0
    while binary > 0:
        y = binary % 10
        binary = binary // 10
        sum = sum + y * (2 ** counter)
        counter += 1
    # print("Your result in decimal is: ", sum)
    return sum


def main():
    # Create a Camera object
    zed = sl.Camera()
    pcd = PointCloud()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.UNIT_MILLIMETER  # Use milliliter units (for depth measurements)

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.SENSING_MODE_STANDARD  # Use STANDARD sensing mode

    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    while 1:
        # A new image is available if grab() returns SUCCESS
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.VIEW_LEFT)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, sl.MEASURE.MEASURE_XYZRGBA)

            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            x = round(image.get_width() / 2)
            y = round(image.get_height() / 2)
            err, point_cloud_value = point_cloud.get_value(x, y)

            numPoints = 1280*720*3
            numColors = 1280*720*1
            points = np.arange(numPoints).reshape((-1, 3))
            colors = np.arange(numColors).reshape((-1, 1))

            begin = datetime.datetime.now()

            my = point_cloud.get_data()

            aa = my.flatten()
            aa.resize((numColors,4))

            aa = np.array(aa)


            filPoints = aa[~np.isnan(aa).any(axis=1)]

            filPoints = filPoints[::15,:]

            points = filPoints[:,0:3].reshape(-1,3)
            colors = filPoints[:,3].reshape(-1,1)
            print(colors.shape)
            print(points.shape)


            # filPoints = points[~np.isnan(points).any(axis=1)]

            pcd.points = Vector3dVector(points)
            pcd.paint_uniform_color([0,1,0])
            for i in range(len(colors)):
                A, R, G, B = float2decimal(colors[i][0])
                print("values of BGR: ", B,G,R)
                pcd.colors[i] = [B/255, G/255, R/255]

            end = datetime.datetime.now()
            k = end - begin

            draw_geometries([pcd])

            distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])

            if not np.isnan(distance) and not np.isinf(distance):
                distance = round(distance)
                print("Distance to Camera at ({0}, {1}): {2} mm\n".format(x, y, distance))
                # Increment the loop
                # i = i + 1
            else:
                print("Can't estimate distance at this position, move the camera\n")
            sys.stdout.flush()

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()