
import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl
import numpy as np
import progressbar
import tifffile
import scipy.ndimage
import matplotlib.pyplot as plt
import os.path
import os
import skimage.measure

def get_image(zed, point_cloud, medianFrames=1, components=[2]):

    """
    This function is giving an average value of the components, X, Y or Z 
    obtained by a certain number of sequentialy acquired frames.

    This helps to stabilize the coordinates acquired, in case of flickering for instance.

    Args:
      zed: initialized and opened zed camera
      point_cloud: initialized point cloud of the zed Camera
      medianFrames: Number of sequentialy acquired Frames for the average value generation
      components: List of values 0,1 or 2 for respectively X,Y and Z coordinates.

    Returns:
      The median value of the coordinates acquired.
    """


    stack_of_images = []
    for n in progressbar.progressbar(range(medianFrames)):
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,sl.MEM.CPU, zed.get_camera_information().camera_resolution)
            point_cloud_np = point_cloud.get_data()
            stack_of_images.append(point_cloud_np)      
        else:
            print(":(")
            return None
    stack_of_images = np.array(stack_of_images)
    stack_of_images[not np.isfinite] = np.nan
    median = np.nanmedian(stack_of_images, axis=0)

    return median[:,:,components]

def pause():
    programPause = input("Press the <ENTER> key to continue...")

def get_Disk_Postion(imageZoffset, newImageXYZ,ROI,CALIB_Z_THRESHOLD_M,RADIUS_TOLERANCE,RADIUS_PERI_THRESHOLD_PX):

    """
    This function is giving us the coordinates of the center of a circular object, a CD for instance.
    By acquiring the coordinates of a certain number of points located on a plane,
    we will be able to calibrate the system.

    Args:
      imageZoffset: The offset of the Z coordinates (Zcoordinates - Background)
      newImageXYZ: The X,Y and Z coordinates of the image
      ROI: The region of Interest 
      CALIB_Z_THRESHOLD_M: The Z threshold corresponding to the Z offset of the object we try to detect
      RADIUS_PERI_THRESHOLD_PX: The threshold to detect a round object of a given radius.

    Returns:
      The X,Y,Z coordinates of the center of the CD.
    """

    
    # Segmentation of objects wich appeared into the scene with a Z-difference of at least : CALIB_Z_THERSHOLD_M 
    binaryCalib = imageZoffset[ROI] > CALIB_Z_THRESHOLD_M 
    objects = scipy.ndimage.label(binaryCalib)[0]

    # Acquisition of properties
    properties = skimage.measure.regionprops(objects)

    # Circularity Test
    circlesBool = []
    print("Starting Circularity Test ...")
    for label in range(objects.max()):

        # Perimeter and Area acquisition
        peri = properties[label].perimeter
        area = properties[label].area

        # Calculation of the radius
        rPeri = peri/2/np.pi
        rArea = (area/np.pi)**0.5
    
        # Circularity test
        isCircle = np.isclose(rPeri, rArea, atol=rArea*RADIUS_TOLERANCE) and rPeri > RADIUS_PERI_THRESHOLD_PX
        circlesBool.append(isCircle)
        print(f"rPeri {rPeri:.2f} -- rArea {rArea:.2f} -- {isCircle}")
    circlesBool = np.array(circlesBool)

    # Detection of a circular object
    if circlesBool.sum() == 1:
        print("A suitable disk has been detected.")
        label = np.where(circlesBool)[0][0]
        centroidPx = properties[label].centroid

        # Transformation of pixel coordinates into XYZ coordinates, taking sequentialy each dimension
        coordsXYZm = []

        # Formating the Px coords in the good format for map_coordinates
        coordsPx = np.array([[centroidPx[0]], [centroidPx[1]]])
        for d in range(3):
            #print(f"{newImageXYZ[ROI[0], ROI[1], d][int(coordsPx[0]//1), int(coordsPx[1]//1)]}")

            # unsophisitcated D_position
            # D_position = newImageXYZ[ROI[0], ROI[1], d][int(coordsPx[0]//1), int(coordsPx[1]//1)]

            D_position = scipy.ndimage.map_coordinates(newImageXYZ[ROI[0], ROI[1], d], coordsPx)[0]
            coordsXYZm.append(D_position)
    elif circlesBool.sum() == 0:
        print("No suitable objects found")
        return None
    else:
        print("More than one suitable object found, need to select best one...")
        return None
    print(f"Found your disk at (x,y,z in meters): {coordsXYZm}")
    return coordsXYZm