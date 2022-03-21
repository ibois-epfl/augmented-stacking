#!/usr/bin/env python


'''
This Code will give the calibration points of the ZED Camera:

1. By acquiring first the Z Cordinates of the Empty scene 
2. By measuring different positions of a given circular object (ex: CD) move on a specific grid.

usage: calib.py [-h] [-cpts NB_CALIB_PTS] [-zT CALIB_Z_THRESHOLD] [-rT RADIUS_THRESHOLD_PX] [-s STARTING_POINT] [-v VISUALIZE]
Create a calibration image.
optional arguments:
  -h, --help                                                        Show this help message and exit.
  -cpts NB_CALIB_PTS, --nbCalibPts NUMBER_OF_CALIB_PTS              Number of Calibration Points.
  -zT CALIB_Z_THRESHOLD, --calibZThresh CALIB_Z_THRESHOLD_M         Approximate height of the element used for the calibration in meters.
  -rT RADIUS_THRESHOLD_PX, --rThresh RADIUS_PERI_THRESHOLD_PX       Radius in px of the element used for the calibration.
  -s STARTING_POINT, --sPoint STARTING_POINT                        Number of the starting point, can be used if one acquisition failed, instead of doing all again start from last point.
  -v VISUALIZE, --visualize VISUALIZE                               Boolean for visualising each acquired image. 
  -sp SAVING_PATH --savePath SAVING_PATH                            Path where the point clouds are saved.
'''

import sys

from xmlrpc.client import Boolean
import json
import pyzed.sl as sl
import numpy as np
import tifffile
import scipy.ndimage
import matplotlib.pyplot as plt
import os.path
import os
from PIL import Image
from PIL import ImageTk
import skimage.measure
from Calibration_functions import get_image 
from Calibration_functions import get_Disk_Position
from Calibration_functions import pause
from Calibration_functions import draw_grid
from Calibration_functions import display_calibration
from Calibration_functions import calculate_3D_2D_matrix
from Calibration_functions import get_3D_2D_matrix
import argparse

# Number of frames we use for the background acquisition
NUMBER_OF_AVERAGE_FRAMES = 64

# Region of Interest can change if the camera moves
ROI = [slice(100, 600), slice(400, 1000)]

# Radius tolerance when comparing the radius given from the area and the perimeter
RADIUS_TOLERANCE = 1



def main(CALIB_Z_THRESHOLD_M,RADIUS_PERI_THRESHOLD_PX,STARTING_POINT,VISUALIZE):

    _utils_path = ".utils/"
    _grid_path = _utils_path + "grid/"
    _calib_img_path = _grid_path + "calibration_image.png"
    _calib_2D_pixel_path = _grid_path + "2D_pixel_coordinates.npy"
    _calib_background_path = _utils_path + "Background.tiff"

    _points_path = _utils_path + "points/"
    _coordinates_path = _points_path + "coordinates/"
    _calib_temp_image_path = _points_path + "imgs/"
    _calib_3D_path = _points_path + "calib_3D_points.npy"
    _calibration_information_path = _utils_path + "calibration_info.json" #TODO: Transform json into yaml file.

    os.makedirs(_utils_path,exist_ok=True)
    os.makedirs(_grid_path,exist_ok=True)
    os.makedirs(_calib_2D_pixel_path,exist_ok=True)
    os.makedirs(_calib_3D_path,exist_ok=True)
    os.makedirs(_calib_img_path,exist_ok=True)
    os.makedirs(_calib_temp_image_path,exist_ok=True)
    os.makedirs(_calibration_information_path,exist_ok=True)
    os.makedirs(_coordinates_path,exist_ok=True)
    os.makedirs(_points_path,exist_ok=True)
    os.makedirs(_calib_background_path,exist_ok=True)

    ###########################################################################################################################################
    ### 1. Display of the Calibration Grid
    ###########################################################################################################################################



    print("We will now generate the Calibration grid.")
    _nb_lines_Y = int(input("Number of pints in X direction:"))
    _nb_lines_X = int(input("Number of points in Y direction:"))
    NUMBER_OF_CALIB_PTS = _nb_lines_X * _nb_lines_Y

    draw_grid(_calib_img_path,_calib_2D_pixel_path,nb_lines_X=_nb_lines_X,nb_lines_Y=_nb_lines_Y)

    print("We will now project the calibration grid, make sure you expend your screen on the projector.\nSo you can see both the terminal on your screen and the calibration image projected on the floor.")
    print("If you want to exit the full screen mode just press the <ESC> key, but this will close the image.")
    pause()
    display_calibration(_calib_img_path)
    
    ###########################################################################################################################################
    ### 2.1 Setting ZED params
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


    ###########################################################################################################################################
    ### 2.2. Setting point cloud params 
    ###########################################################################################################################################
    
    point_cloud = sl.Mat(zed.get_camera_information().camera_resolution.width, 
                         zed.get_camera_information().camera_resolution.height,
                         sl.MAT_TYPE.F32_C4,
                         sl.MEM.CPU)


    ###########################################################################################################################################
    ### 3. Acquiring Background - Empty the scene
    ###########################################################################################################################################

    # Check if the empty background image exists
    if not os.path.exists(_calib_background_path):
        print(f"{_calib_background_path} not found, acquiring {NUMBER_OF_AVERAGE_FRAMES} frames.")
        print("Make sure the scene is Empty.")
        pause()

        # Background average depth 
        print("Background acquisition ...")
        background = get_image(zed,point_cloud,medianFrames=NUMBER_OF_AVERAGE_FRAMES, components=[2])[:,:,0]
        tifffile.imwrite(_calib_background_path, background)
    else:
        print("Loading previous Background.tiff")
        background = tifffile.imread(_calib_background_path)
        print(f"I loaded a background image with shape: {background.shape}")

        
    ###########################################################################################################################################
    ### 4. Acquiring Positions - Place the CD on the scene 
    ###########################################################################################################################################

    # The CD has to be at a minimum height of calibZThresholdM in meters
    print("Acquiring Positions ...")
    Stack_coordsXYZm = []
    Stack_coordsPixs = []
    i=STARTING_POINT
    while i < NUMBER_OF_CALIB_PTS:
        print(f"Put the CD into the Scene. On position {i+1}.")
        pause()
        print("Acquiring image ...")
        newImageXYZ = get_image(zed,point_cloud,
                                medianFrames=NUMBER_OF_AVERAGE_FRAMES,
                                components=[0,1,2])
        newImageZoffset = newImageXYZ[:,:,2]-background
        tifffile.imwrite(f"{_calib_temp_image_path}/Image_position_Z_{i+1}.tiff", newImageXYZ[:,:,2][ROI]-background[ROI])
        if VISUALIZE:
            ### Visualize Offset Image
            plt.imshow(newImageXYZ[:,:,2], vmin=-0.2 , vmax=0.2, cmap='coolwarm')
            plt.show()
            ## Visualize Offset Image ROI
            plt.imshow(newImageXYZ[:,:,2][ROI]-background[ROI], vmin=-0.2 , vmax=0.2, cmap='coolwarm')
            plt.show()
        print("Acquiring position ...")
        coordsXYZm,CoordsPx = get_Disk_Position(newImageZoffset,newImageXYZ,ROI,CALIB_Z_THRESHOLD_M,RADIUS_TOLERANCE,RADIUS_PERI_THRESHOLD_PX)
        if not coordsXYZm == None:
            if not np.isnan(coordsXYZm[0]):
                np.save(f"{_coordinates_path}Image_position_{i+1}.npy", np.array(coordsXYZm))
                Stack_coordsXYZm.append(coordsXYZm)
                Stack_coordsPixs.append(CoordsPx)
                i+=1
            else:
                print("nan value encountered please try again")
        else:
            print(f"New acquisition of point {i+1}")
    

    ## Gathering all points into one file
    if i == NUMBER_OF_CALIB_PTS:
        calibPointsXYZ = np.array(Stack_coordsXYZm)
        np.save(_calib_3D_path,calibPointsXYZ)
    
    ###########################################################################################################################################
    ### 5. Obtain the pixel to meter and meter to pixel conversion for ROI
    ###########################################################################################################################################
    
    ## Obtain the conversion between the 3D point cloud and the pixel ROI
    XYZ_Pt1 = np.load(f"{_coordinates_path}Image_position_1.npy") 
    XYZ_Pt2 = np.load(f"{_coordinates_path}Image_position_{_nb_lines_X}.npy")
    Pixs_Pt1 = Stack_coordsPixs[0][0]
    Pixs_Pt2 = Stack_coordsPixs[_nb_lines_X][0]

    Distance_m = np.abs(XYZ_Pt1[0]-XYZ_Pt2[0])
    Distance_px = int(np.abs(Pixs_Pt1[0]-Pixs_Pt2[0]))

    print(f"{Distance_m}m in the 3D point cloud corresponds to {Distance_px}px on the acquired image.")

    # Closing camera
    zed.close()

    ###########################################################################################################################################
    ### 6. Estimate the 3D 2D matrix 
    ###########################################################################################################################################

    From_3D_2D_matrix = calculate_3D_2D_matrix(_calib_2D_pixel_path,_calib_3D_path)
    Data = {"3D_2D_Matrix":From_3D_2D_matrix,"ROI_info":{"Distance_m":Distance_m,"Distance_px":Distance_px}}
    
    data_file = open(_calibration_information_path, "w")
    json.dump(Data, data_file)
    data_file.close()

    P = get_3D_2D_matrix(_calibration_information_path)
    print(P)



if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description="Get the Calibration Points",
                                   formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        '-zT', '--calibZThresh',
        help="Approximate height (float) of the element used for the calibration in meters.",
        type=float,
        default=0.08
    )
    parser.add_argument(
        '-rT', '--rThresh',
        help="Radius in px of the element used for the calibration.",
        type=int,
        default=20
    )
    parser.add_argument(
        '-s', '--sPoint',
        help="Number of the starting point, can be used if one acquisition failed, instead of doing all again start from last point.",
        type=int,
        default=0
    )
    parser.add_argument(
        '-v', '--visualize',
        help="Boolean for visualising each acquired image.",
        type=bool,
        default=True
    )

    args = parser.parse_args()

    
    main(args.calibZThresh, args.rThresh,args.sPoint,args.visualize)
