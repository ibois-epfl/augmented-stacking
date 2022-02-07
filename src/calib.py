#!/usr/bin/env python

import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl
import numpy as np
import progressbar

# test

NUMBER_OF_AVERAGE_FRAMES = 64
#       YYYYYY            XXXXXXXX
#ROI = [slice(200, 600), slice(400, 900)] ## Dont move the machine!!
ROI = [slice(200, 500), slice(400, 900)] ## Dont move the machine!!
calibZThresholdM = 0.08
rPeriThresholdPx = 10

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
    
    # Open ZED
    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()
    camera_info = zed.get_camera_information()
    print("POP: ZED camera opened, serial number: {0}".format(camera_info.serial_number))

    # Create OpenGL viewer, launch and set Viewer Camera params
    #viewer = gl.GLViewer()
    #print("POP: Running Augmented Stacking depth capture GL viewer... Press 'Esc' to quit")

    camera_model = zed.get_camera_information().camera_model
    #viewer.init(len(sys.argv), sys.argv, camera_model, zed.get_camera_information().camera_resolution)
    
    #viewer.bckgrnd_clr = np.array([170/255., 100/150., 180/255.])
    #viewer.camera.position_.init_vector(5., 0., -2.) # // TEST
    #viewer.camera.setRotation(sl.Rotation().set_rotation_vector(0., 0., 90.)) # 0,90,180 // TEST


    # Set point cloud object params from ZED frame
    point_cloud = sl.Mat(zed.get_camera_information().camera_resolution.width, 
                         zed.get_camera_information().camera_resolution.height,
                         sl.MAT_TYPE.F32_C4,
                         sl.MEM.CPU)

    import tifffile
    import numpy
    import scipy.ndimage
    import matplotlib.pyplot as plt
    import os.path
    import skimage.measure
    # Main loop
    #while viewer.is_available():

    def oneImage(medianFrames=1, components=[2]):
        stack = []
        for n in progressbar.progressbar(range(medianFrames)):
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,sl.MEM.CPU, zed.get_camera_information().camera_resolution) # for color: sl.MEASURE.XYZRGBA
                #viewer.updateData(point_cloud)

                point_cloud_np = point_cloud.get_data()

                stack.append(point_cloud_np[:,:,components])
                
                # print(point_cloud_np)
                # for i in range(point_cloud_np.shape[2]):
                #     print("writing")
                #     tifffile.imsave(f"{i}.tiff", point_cloud_np[:,:,i])
                
            else:
                print(":(")
                return None
        stack = np.array(stack)
        stack[not numpy.isfinite] = numpy.nan
        median = numpy.nanmedian(stack, axis=0)
        return median[:,:,components]


    if not os.path.exists('zMedian.tiff'):
        print(f"zMedian not found, acquiring {NUMBER_OF_AVERAGE_FRAMES} frames...")
        # Assemble depth average
        zMedian = oneImage(medianFrames=NUMBER_OF_AVERAGE_FRAMES, components=[2])
        print(zMedian.shape)
        tifffile.imsave("zMedian.tiff", zMedian)

    else:
        print("Loading prevous zMedian.tiff")
        zMedian = tifffile.imread('zMedian.tiff')

    
    newImageXYZ = oneImage(medianFrames=32, components=[0,1,2])
    print(newImageXYZ.shape)
    newImageZoffset = newImageXYZ[:,:,2]-zMedian
    # plt.imshow(newImage, vmin=-0.2 , vmax=0.2, cmap='coolwarm')
    # plt.show()
    
    tifffile.imsave("newImage.tiff", newImageXYZ)

    # plt.imshow(newImage[ROI], vmin=-0.2 , vmax=0.2, cmap='coolwarm')
    # plt.show()

    binaryCalib = newImageZoffset[ROI] > calibZThresholdM 
    objects = scipy.ndimage.label(binaryCalib)[0]

    properties = skimage.measure.regionprops(objects)

    circlesBool = []
    for label in range(1, objects.max()+1):
        peri = properties[label-1].perimeter
        area = properties[label-1].area

        rPeri = peri/2/numpy.pi
        rArea = (area/numpy.pi)**0.5
        
        isCircle = numpy.isclose(rPeri, rArea, atol=rArea/4) and rPeri > rPeriThresholdPx
        circlesBool.append(isCircle)
        print(f"rPeri {rPeri:.2f} -- rArea {rArea:.2f} -- {isCircle}")
    circlesBool = numpy.array(circlesBool)

    print(circlesBool.sum())
    if circlesBool.sum() == 1:
        #print(numpy.where(circlesBool)[0][0])
        centroidPx = properties[numpy.where(circlesBool)[0][0]].centroid
        print(centroidPx)
        coordsXYZm = []
        for d in range(3):
            #print(newImageXYZ[ROI[0], ROI[1], d].shape)
            coordsXYZm.append(scipy.ndimage.map_coordinates(newImageXYZ[ROI[0], ROI[1], d], numpy.array([[centroidPx[0]], [centroidPx[1]]]))[0])
    elif circlesBool.sum() == 0:
        print("No suitable objects found")
    else:
        print("More than one suitable object found, need to select best one...")

    print(f"Found your disk at (x,y,z in meters): {coordsXYZm}")

    # for label in 

    plt.imshow(objects, cmap='coolwarm')
    plt.show()

    # When "Esc" is pressed, we close viewer and zed
    #viewer.exit()
    zed.close()


if __name__ == "__main__":
    main()

