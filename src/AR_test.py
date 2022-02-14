   
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


imXYZ = get_image(zed, point_cloud, medianFrames=16, components=[0,1,2])
plt.imshow(imXYZ[:,:,2][ROI])
plt.show()
posMax = numpy.where(imXYZ[:,:,2][ROI] == numpy.nanmax(imXYZ[:,:,2][ROI]))
print("This is the highest pixel in the ROI")
zed.close()

XYZpos = []
for i in range(3):
    XYZpos.append(imXYZ[:, :, i][ROI][posMax[0][0], posMax[1][0]])
#XYZpos = imXYZ[posMax[0][0], posMax[1][0], :]
XYZpos = numpy.array(XYZpos)
print(XYZpos)

RHS = numpy.dot(numpy.dot(K, Rt), numpy.array([XYZpos[0], XYZpos[1], XYZpos[2], 1]).T)/s
print(f"This pixel should be illumiated: {RHS[0:2]}")

outputHack = numpy.zeros((1080, 1920), dtype='<u1')
outputHack[int(RHS[1]-5):int(RHS[1]+5), int(RHS[0]-5):int(RHS[0]+5)] = 255
tifffile.imsave("urgh.tif", outputHack)