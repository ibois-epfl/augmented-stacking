import pyzed.sl as sl
import numpy as np
import open3d as o3d

# open camera
# capture point cloud
# rearrange point cloud points following center of image as origin
# export point cloud as .ply format

# Set ZED params
init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, # HD720 | 1280*720
                            camera_fps=30, # available framerates: 15, 30, 60 fps
                            depth_mode=sl.DEPTH_MODE.QUALITY, # posible mods: sl.DEPTH_MODE.PERFORMANCE/.QUALITY/.ULTRA
                            coordinate_units=sl.UNIT.METER,
                            coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP, # sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP
                            sdk_verbose = True, # Enable verbose logging
                            depth_minimum_distance=0.3, # Enable capture from 30 cm
                            depth_maximum_distance=3.0 # Enable capture up to 3m
                            ) 


# Open ZED and catch error
zed = sl.Camera()
status = zed.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()
camera_info = zed.get_camera_information()
print("ZED camera opened, serial number: {0}".format(camera_info.serial_number))

# Setting an empty point cloud
point_cloud = sl.Mat(zed.get_camera_information().camera_resolution.width, 
                         zed.get_camera_information().camera_resolution.height,
                         sl.MAT_TYPE.F32_C4,
                         sl.MEM.CPU)

# Capture point cloud
if zed.grab() == sl.ERROR_CODE.SUCCESS:
    zed.retrieve_measure(point_cloud,
                            sl.MEASURE.XYZRGB,
                            sl.MEM.CPU,
                            zed.get_camera_information().camera_resolution)
    point_cloud_np = point_cloud.get_data()


# point3D = point_cloud.get_value(i,j)
# x = point3D[0]
# y = point3D[1]
# z = point3D[2]
# color = point3D[3]

# print(point_cloud_np)
print(type(point_cloud_np))
print(point_cloud_np.shape)

point_cloud_np[:,:,1:3]

point_cloud_o3d = o3d.geometry.PointCloud()
point_cloud_o3d.points = o3d.utility.Vector3dVector()
 

# Close the camera
zed.close()



