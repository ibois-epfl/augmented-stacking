#!/usr/bin/env python

import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl
import numpy as np


def main():

    print("Running Augmented Stacking depth capture GL viewer... Press 'Esc' to quit")

    # Set ZED params
    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, # HD720 | 1280*720
                             camera_fps=30, # available framerates: 15, 30, 60 fps
                             depth_mode=sl.DEPTH_MODE.QUALITY, # posible mods: sl.DEPTH_MODE.PERFORMANCE/.QUALITY/.ULTRA
                             coordinate_units=sl.UNIT.METER,
                             coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP, # sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP
                             sdk_verbose = True) # Enable verbose logging
    
    # Open ZED
    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    
    # Create OpenGL viewer, launch and set Viewer Camera params
    viewer = gl.GLViewer()

    camera_model = zed.get_camera_information().camera_model
    viewer.init(len(sys.argv), sys.argv, camera_model, zed.get_camera_information().camera_resolution)
    
    viewer.bckgrnd_clr = (0, 0, 0)
    viewer.camera.position_.init_vector(5, 0, -3) # // TEST
    viewer.camera.setRotation(sl.Rotation().set_rotation_vector(0, 0, 180)) # 0,90,180 // TEST



    # Set point cloud object params from ZED frame
    point_cloud = sl.Mat(zed.get_camera_information().camera_resolution.width, 
                         zed.get_camera_information().camera_resolution.height,
                         sl.MAT_TYPE.F32_C4,
                         sl.MEM.CPU)

    # Feed point cloud to OpenGL viewer and refresh
    while viewer.is_available():
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,sl.MEM.CPU, zed.get_camera_information().camera_resolution) # for color: sl.MEASURE.XYZRGBA
            viewer.updateData(point_cloud)

    # When "Esc" is pressed, we close viewer and zed
    viewer.exit()
    zed.close()


if __name__ == "__main__":
    main()

