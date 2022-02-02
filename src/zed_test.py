#!/usr/bin/env python

import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl


def main():

    print("Running Augmented Stacking depth capture GL viewer... Press 'Esc' to quit")

    # Set ZED params
    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, # HD720 | 1280*720
                             camera_fps=30, # available framerates: 15, 30, 60 fps
                             depth_mode=sl.DEPTH_MODE.PERFORMANCE, # for higher res sl.DEPTH_MODE.PERFORMANCE
                             coordinate_units=sl.UNIT.METER,
                             coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
    
    # Open ZED
    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    # Retrieve ZED camera model
    camera_model = zed.get_camera_information().camera_model
    
    # Create OpenGL viewer an launch it
    viewer = gl.GLViewer()
    viewer.init(len(sys.argv), sys.argv, camera_model, zed.get_camera_information().camera_resolution)

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

