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
import shutil
from xmlrpc.client import Boolean
import yaml
import pyzed.sl as sl
import numpy as np
import tifffile
import matplotlib.pyplot as plt
import os.path
import os
from calibration_functions import get_image 
from calibration_functions import get_Disk_Position
from calibration_functions import pause
from calibration_functions import draw_grid
from calibration_functions import display_calibration
from calibration_functions import calculate_3D_2D_matrix
from calibration_functions import get_3D_2D_matrix
_root_file = os.path.dirname(__file__)
if _root_file == '':
    sys.path.append(_root_file+"../util")
else:
    sys.path.append(_root_file+"/../util")
import terminal
import argparse
import glob
from tqdm import tqdm

# Number of frames we use for the background acquisition
NUMBER_OF_AVERAGE_FRAMES = 64

# Region of Interest can change if the camera moves
ROI = [slice(100, 600), slice(400, 1000)]

# Radius tolerance when comparing the radius given from the area and the perimeter
RADIUS_TOLERANCE = 1



def main(OLD_ACQUISITION,CALIB_Z_THRESHOLD_M,RADIUS_PERI_THRESHOLD_PX,STARTING_POINT,VISUALIZE):

    root_file = os.path.dirname(__file__)
    if root_file == "":
        _utils_path = root_file + "utils/"
    else:
        _utils_path = root_file + "/utils/"

    _grid_path = _utils_path + "grid/"
    _calib_img_path = _grid_path + "calibration_image.png"
    _calib_2D_pixel_path = _grid_path + "2D_pixel_coordinates.npy"
    _calib_2D_camera_pixel_path = _grid_path + "camera_2D_pixel_coordinates.npy"
    _calib_background_path = _utils_path + "Background.tiff"
    _display_calib_function_path = root_file + "/display_calib_grid.py"

    _points_path = _utils_path + "points/"
    _coordinates_path = _points_path + "coordinates/"
    _calib_temp_image_path = _points_path + "imgs/"
    _calib_3D_path = _points_path + "calib_3D_points.npy"
    _calibration_information_path = _utils_path + "calibration_info.yaml" 

    os.makedirs(_utils_path,exist_ok=True)
    os.makedirs(_grid_path,exist_ok=True)
    os.makedirs(_calib_temp_image_path,exist_ok=True)
    os.makedirs(_coordinates_path,exist_ok=True)
    os.makedirs(_points_path,exist_ok=True)

    ###########################################################################################################################################
    ### 0. Selecting the parameters
    ###########################################################################################################################################



    print("\n##########################################\n## 0. Initialisation \n##########################################")
    
    ## number of calibration points
    print(f"This calibration will create a folder {_utils_path} where every thing will be stored.")
    print("\nThe number of calibration points have to be at least 8, otherwise the calibration wont be precise enough.")
    print("The more points you have the better the calibration. I suggest you to take 3*3 or 4*3 points.")
    print("For more precision: use the distortion option of the projector and make the corners angle right.")
    _nb_lines_Y = int(terminal.user_input("Number of points in X direction:"))
    _nb_lines_X = int(terminal.user_input("Number of points in Y direction:"))
    NUMBER_OF_CALIB_PTS = _nb_lines_X * _nb_lines_Y
    if NUMBER_OF_CALIB_PTS > 7:
        print(f"This makes a total of {NUMBER_OF_CALIB_PTS} points.")
    else:
        # At least 8 points are needed
        print(f"Please select more points, at least 8.")
        exit()

    ## Using old Acquisition
    if os.path.exists(_calib_2D_pixel_path) and os.path.exists(_calib_3D_path) and os.path.exists(_calib_2D_camera_pixel_path):
        print(f"\nAcquisition files already exist.\n\n - {_calib_2D_pixel_path}\n - {_calib_3D_path}\n - {_calib_2D_camera_pixel_path}")
        if os.path.exists(_calib_background_path):
            print(f"\nThe following background was used:")
            pause()
            background = tifffile.imread(_calib_background_path)
            plt.imshow(background)
            plt.title(f"{_calib_background_path}")
            plt.show()
            answer = None
            while not answer in ["Y","n"]:
                answer = str(terminal.user_input("\n Having seen the background.\n Do you want to reuse the last acquisition and skip the new acquisition process ? (Y/n)\n>> "))
            if answer == "Y":
                print("You have chosen to keep the old acquired points.")
                OLD_ACQUISITION = True
            else:
                print("You have chosen to do a new acquisition.")
                OLD_ACQUISITION = False
    ## Consistency btw number_of_calib_pts and nb_saved_files.
    if OLD_ACQUISITION:
        list_3D = glob.glob(f"{_coordinates_path}Image_position_*.npy")
        list_2D = glob.glob(f"{_coordinates_path}Camera_px_position_*.npy")
        if not (len(list_2D) == NUMBER_OF_CALIB_PTS and len(list_3D) == NUMBER_OF_CALIB_PTS):
            print(f"\n It seems the number of points saved are not equal the amount you asked for.\n\n  - Asked points: {NUMBER_OF_CALIB_PTS} \n  - 2D points: {len(list_2D)} \n  - 3D points: {len(list_3D)}")
            answer = None
            while not answer in ["Y","n"]:
                answer = terminal.user_input("Do you want to continue, making a new acquisition ? (Y/n)\n>> ")
            if answer == "Y":
                OLD_ACQUISITION = False
            else:
                exit()
    pause()

    
    ###########################################################################################################################################
    ### 1. Display and Generation of the Calibration Grid
    ###########################################################################################################################################

    print("\n##########################################\n## 1. Calibration Grid Generation \n##########################################")

    draw_grid(_calib_img_path,_calib_2D_pixel_path,nb_lines_X=_nb_lines_X,nb_lines_Y=_nb_lines_Y)

    # displaying the grid
    print("\n##########################################\n## 2. Calibration Grid Display \n##########################################")
    print("\n A tkinter window will appear, place it on the projectors display.\n\n - <F11>:  Toogle Full screen mode.\n - < q >: Close the window.\n")
    pause()
    threaded_app = display_calibration(_calib_img_path)

    confirmation = None
    while not confirmation in ["y"]:
        confirmation = terminal.user_input("\nConfirmation:  Grid is on place. \n>> Type <y> and Press ENTER to Continue ...")

    ###########################################################################################################################################
    ### 2.1 Setting ZED params if New Acquisition is requested
    ###########################################################################################################################################


    if not OLD_ACQUISITION:
        try:

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
                terminal.error_print(repr(status))
                threaded_app.error_down()
                exit()
            camera_info = zed.get_camera_information()
            print("\nPOP: ZED camera opened, serial number: {0}".format(camera_info.serial_number))


            ###########################################################################################################################################
            ### 2.2. Setting point cloud params 
            ###########################################################################################################################################
            
            point_cloud = sl.Mat(zed.get_camera_information().camera_resolution.width, 
                                zed.get_camera_information().camera_resolution.height,
                                sl.MAT_TYPE.F32_C4,
                                sl.MEM.CPU)
        except Exception as e:
            terminal.error_print(e)
            threaded_app.error_down()
            exit()

    ###########################################################################################################################################
    ### 3. Acquiring Background - Empty the scene
    ###########################################################################################################################################
    print("\n##########################################\n## 3. Background Acquisition \n##########################################")
    try:
        if not OLD_ACQUISITION:
            # Check if the empty background image exists
            print(f"Acquiring {NUMBER_OF_AVERAGE_FRAMES} frames.")
            print("Make sure the scene is Empty.")
            pause()

            # Background average depth 
            print("Background acquisition ...")
            background = get_image(zed,point_cloud,medianFrames=NUMBER_OF_AVERAGE_FRAMES, components=[2])[:,:,0]
            tifffile.imwrite(_calib_background_path, background)
        else:
            print("\nYou have chosen to keep the old acquired points, no background is required.")
            
    except Exception as e:
        terminal.error_print(e)
        threaded_app.error_down()
        exit()
        
    ###########################################################################################################################################
    ### 4. Acquiring Positions - Place the CD on the scene 
    ###########################################################################################################################################


    print("\n##########################################\n## 4. Calibration Points Acquisition \n##########################################")
    ## Delete old files ?

    try:
        if not OLD_ACQUISITION:
            print(f"\n You are about to Acquire new points. The following folder will be deleted :\n\n  {_coordinates_path}")
            answer = None 
            while not answer in ["Y","n"]:
                answer = terminal.user_input("Do you want to continue ? (Y/n)\n>> ")
            if answer:
                shutil.rmtree(_coordinates_path)
                os.makedirs(_coordinates_path)
            else:
                print("You exited the program.")
                exit()
            print("\nDuring the Acquisition of the images you should not enter the scene.")

            # The CD has to be at a minimum height of calibZThresholdM in meters
            Stack_coordsXYZm = []
            Stack_coordsPixs = []
            i=STARTING_POINT
            while i < NUMBER_OF_CALIB_PTS:
                print(f"\nPut the CD into the Scene. On position {i+1}.")
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
                coordsXYZm,coordsPx = get_Disk_Position(newImageZoffset,newImageXYZ,ROI,CALIB_Z_THRESHOLD_M,RADIUS_TOLERANCE,RADIUS_PERI_THRESHOLD_PX)
                if not coordsXYZm == None:
                    if not np.isnan(coordsXYZm[0]):
                        np.save(f"{_coordinates_path}Image_position_{i+1}.npy", np.array(coordsXYZm))
                        np.save(f"{_coordinates_path}Camera_px_position_{i+1}.npy",np.array(coordsPx))
                        Stack_coordsXYZm.append(coordsXYZm)
                        Stack_coordsPixs.append(coordsPx)
                        i+=1
                    else:
                        print("nan value encountered please try again")
                else:
                    print(f"New acquisition of point {i+1}")

            ## Gathering all points into one file
            if i == NUMBER_OF_CALIB_PTS:
                calibPointsXYZ = np.array(Stack_coordsXYZm)
                np.save(_calib_3D_path,calibPointsXYZ)
                cameraPointsPx = np.array(Stack_coordsPixs)
                np.save(_calib_2D_camera_pixel_path,cameraPointsPx)
            
            # Closing camera
            zed.close()
        else:
            Stack_coordsPixs = []
            Stack_coordsXYZm = []
            print("\n Loading the last acquired points.")
            for i in tqdm(range(NUMBER_OF_CALIB_PTS)):
                coordsXYZm = np.load(f"{_coordinates_path}Image_position_{i+1}.npy")
                coordsPx = np.load(f"{_coordinates_path}Camera_px_position_{i+1}.npy")
                Stack_coordsXYZm.append(coordsXYZm)
                Stack_coordsPixs.append(coordsPx)
            pause()
    except Exception as e:
        terminal.error_print(e)
        threaded_app.error_down()
        exit()

    ###########################################################################################################################################
    ### 5. Obtain the pixel to meter and meter to pixel conversion for ROI
    ###########################################################################################################################################
    print("\n##########################################\n## 5. 3D to Camera Conversion for ROI definition \n##########################################")

    try:
        ## Obtain the conversion between the 3D point cloud and the pixel ROI
        XYZ_Pt1 = np.load(f"{_coordinates_path}Image_position_1.npy") 
        XYZ_Pt2 = np.load(f"{_coordinates_path}Image_position_{_nb_lines_X}.npy")
        Pixs_Pt1 = np.load(f"{_coordinates_path}Camera_px_position_1.npy") 
        Pixs_Pt2 = np.load(f"{_coordinates_path}Camera_px_position_{_nb_lines_X}.npy")
        Distance_m = np.round(np.sqrt((XYZ_Pt1[0]-XYZ_Pt2[0])**2 + (XYZ_Pt1[1]-XYZ_Pt2[1])**2 + (XYZ_Pt1[2]-XYZ_Pt2[2])**2),2)
        Distance_px = np.round(np.sqrt((Pixs_Pt1[0]-Pixs_Pt2[0])**2 + (Pixs_Pt1[1]-Pixs_Pt2[1])**2),2)
        print(f"\n{Distance_m} m in the 3D point cloud corresponds to {Distance_px} px on the camera image.")
    except Exception as e:
        terminal.error_print(e)
        threaded_app.error_down()
        exit()

    ###########################################################################################################################################
    ### 6. Estimate the 3D 2D matrix 
    ###########################################################################################################################################
    print("\n##########################################\n## 6. Estimation of the 3D to 2D matrix \n##########################################")
    print("\n The following calculation is an optimization of the transformation matrix, between the 3D point cloud and the projected image.")
    pause()
    try:
        From_3D_2D_matrix = calculate_3D_2D_matrix(_calib_2D_pixel_path,_calib_3D_path)
        Data = {"3D_2D_Matrix":From_3D_2D_matrix,"ROI_info":{"Distance_m":float(Distance_m),"Distance_px":float(Distance_px)}}
        data_file = open(_calibration_information_path, "w")
        yaml.dump(Data, data_file)
        data_file.close()
        P = get_3D_2D_matrix(_calibration_information_path)
        print(f"\n >>> Saving 3D-2D matrix in: {_calibration_information_path}")
        print(f"\nThe matrix looks as follows: \n{P}")
    except Exception as e:
        print(e)
        threaded_app.stop()
        exit()
    print("\n\n End of Calibration \n\n")
    threaded_app.error_down()



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

    parser.add_argument(
        '-a', '--oldAcquisition',
        help="Boolean for a new point acquisition.",
        type=bool,
        default=False
    )

    args = parser.parse_args()
    main(args.oldAcquisition,args.calibZThresh, args.rThresh,args.sPoint,args.visualize)
