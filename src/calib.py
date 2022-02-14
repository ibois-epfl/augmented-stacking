#!/usr/bin/env python


'''
This Code will calibrate the ZED camera:

1. By acquiring first the Z Cordinates of the Empty scene 
2. By measuring different positions of a given circular object (ex: CD) move on a specific grid.

'''



import sys
# import ogl_viewer.viewer as gl
# import pyzed.sl as sl
# import numpy as np
# import progressbar
import tifffile
import scipy.ndimage
import matplotlib.pyplot as plt
import os.path
import os
import skimage.measure
from Calibration_functions import get_image 
from Calibration_functions import get_Disk_Position
from Calibration_functions import pause



# Number of frames we use for the background acquisition
NUMBER_OF_AVERAGE_FRAMES = 64

# Region of Interest can change if the camera moves
ROI = [slice(200, 500), slice(400, 900)]

# Z threshold when comparing new image to background
CALIB_Z_THRESHOLD_M = 0.08

# Radius threshold for filtering noise 
RADIUS_PERI_THRESHOLD_PX = 10

# Radius tolerance when comparing the radius given from the area and the perimeter
RADIUS_TOLERANCE = 0.25

# Number of measured points for calibration
startPoint = 0
#startPoint = 9
NUMBER_OF_CALIB_PTS = 100

# Visualize acquisition 

VISUALIZE = True


def main():
    
    ###########################################################################################################################################
    ### Setting ZED params
    ###########################################################################################################################################
    
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


    ######################reboo#####################################################################################################################
    ### Setting point cloud params 
    ###########################################################################################################################################
    
    point_cloud = sl.Mat(zed.get_camera_information().camera_resolution.width, 
                         zed.get_camera_information().camera_resolution.height,
                         sl.MAT_TYPE.F32_C4,
                         sl.MEM.CPU)


    ###########################################################################################################################################
    ### 1. Acquiring Background - Empty the scene
    ###########################################################################################################################################

    # Check if the empty background image exists
    if not os.path.exists('Background.tiff'):
        print(f"Background.tiff not found, acquiring {NUMBER_OF_AVERAGE_FRAMES} frames.")
        print("Make sure the scene is Empty.")
        pause()

        # Background average depth 
        print("Background acquisition ...")
        background = get_image(zed,point_cloud,medianFrames=NUMBER_OF_AVERAGE_FRAMES, components=[2])
        tifffile.imwrite("Background.tiff", background[:,:,0])
    else:
        print("Loading previous Background.tiff")
        background = tifffile.imread('Background.tiff')
        print(f"I loaded a background image with shape: {background.shape}")

        
    ###########################################################################################################################################
    ### 2. Acquiring Positions - Place the CD on the scene 
    ###########################################################################################################################################

    # The CD has to be at a minimum height of calibZThresholdM in meters
    print("Acquiring Positions ...")
    Stack_coordsXYZm = []
    for i in range(startPoint, NUMBER_OF_CALIB_PTS):
        print(f"Put the CD into the Scene. On position {i+1}.")
        pause()
        print("Acquiring image ...")
        newImageXYZ = get_image(zed,point_cloud,medianFrames=32, components=[0,1,2])
        newImageZoffset = newImageXYZ[:,:,2]-background
        tifffile.imwrite(f"Image_position_Z_{i+1}.tiff", newImageXYZ[:,:,2][ROI]-background[ROI])
        tifffile.imwrite(f"Image_position_all_{i+1}.tiff", newImageXYZ)
        if VISUALIZE:
            ### Visualize Offset Image
            #plt.imshow(newImageXYZ, vmin=-0.2 , vmax=0.2, cmap='coolwarm')
            #plt.show()
            ## Visualize Offset Image ROI
            plt.imshow(newImageXYZ[:,:,2][ROI]-background[ROI], vmin=-0.2 , vmax=0.2, cmap='coolwarm')
            plt.show()
        print("Acquiring position ...")
        coordsXYZm = get_Disk_Postion(newImageZoffset, newImageXYZ,ROI,CALIB_Z_THRESHOLD_M,RADIUS_TOLERANCE,RADIUS_PERI_THRESHOLD_PX)
        np.save(f"Image_position_{i+1}.np", np.array(coordsXYZm))
        Stack_coordsXYZm.append(coordsXYZm)

    # Closing camera
    zed.close()


if __name__ == "__main__":
    main()

