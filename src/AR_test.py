   
# Very rough AR test
import sys
import pyzed.sl as sl
import numpy as np
import tifffile
import scipy.ndimage
import matplotlib.pyplot as plt
from Calibration_functions import get_image
import os.path
import os
import skimage.measure
from Calibration_functions import get_Disk_Position 

# Region of Interest can change if the camera moves
ROI = [slice(100, 600), slice(400, 1000)]

def main():
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
    print("POP: ZED camera opened, serial number: {0}".format(camera_info.serial_number))


    ###########################################################################################################################################
    ### Setting point cloud params 
    ###########################################################################################################################################

    point_cloud = sl.Mat(zed.get_camera_information().camera_resolution.width, 
                            zed.get_camera_information().camera_resolution.height,
                            sl.MAT_TYPE.F32_C4,
                            sl.MEM.CPU)


    ###########################################################################################################################################
    ### Acquiring highest point in the ROI
    ###########################################################################################################################################

    
    newImageXYZ = get_image(zed, point_cloud, medianFrames=16, components=[0,1,2])
    CALIB_Z_THRESHOLD_M = 0.08
    RADIUS_TOLERANCE = 0.25
    RADIUS_PERI_THRESHOLD_PX = 10
    print("Loading previous Background.tiff")
    background = tifffile.imread('Background.tiff')[:,:,0]
    print(f"I loaded a background image with shape: {background.shape}")

    # This is the 3D position of that maximum value
    imageZoffset = newImageXYZ[:,:,2] - background
    XYZpos = get_Disk_Position(imageZoffset, newImageXYZ,ROI,CALIB_Z_THRESHOLD_M,RADIUS_TOLERANCE,RADIUS_PERI_THRESHOLD_PX)
    
    zed.close()
    print(XYZpos)

    
    # Loading the P matrix transition between 3D and 2D
    P = np.load("3D_2D_matrix.npy")
    RHS = np.dot(P, np.array([XYZpos[0], XYZpos[1], XYZpos[2], 1]).T)
    print(f"This pixel should be illumiated: {RHS[0:2]}")

    outputHack = np.zeros((1080, 1920,3))
    outputHack[int(RHS[0]-5):int(RHS[0]+5), int(RHS[1]-5):int(RHS[1]+5),1] = 1
    plt.imsave("Max_point.png", outputHack)
    
if __name__ == '__main__':
    main()
