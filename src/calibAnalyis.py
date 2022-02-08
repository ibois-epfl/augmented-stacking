"""
2022-02-08 EA AS DR -- rough analysis of acquired calibration points.
We want to fit (match):
  - projected points on projector (input 1)
  - 3D points recorded at the locations the projector was shining
Result: K projection matrix between 2D pixels and 3D positions in meters

Note:
  xy = pixels
  XY = physical space (meters)
"""
import numpy
import matplotlib.pyplot as plt
import scipy.optimize
import progressbar

ROI = [slice(200, 500), slice(400, 900)]

rotationAngleDegThreshold = 0.00001

GRAPH = False

### Load projector positions in px
proj_xy = numpy.array([[ 479, 268],
          [ 956, 268],
          [1433, 268],
          [ 479, 536],
          [ 956, 536],
          [1433, 536],
          [ 479, 803],
          [ 956, 803],
          [1433, 803]])
#plt.plot(proj_xy[:,0], proj_xy[:,1], 'x')
#plt.xlabel("x [px]")
#plt.ylabel("y [px]")
#plt.title("Your imposed XY positions")
#plt.show()

### Load CORRESPONDING!!! 3D positions of calibration target
calibPointsXYZ = []
for point in range(1,10):
    calibPointsXYZ.append(numpy.load(f"Image_position_{point}.np.npy"))
calibPointsXYZ = numpy.array(calibPointsXYZ)

print(calibPointsXYZ)

#plt.plot(calibPointsXYZ[:,0], calibPointsXYZ[:,1], 'x')
#plt.xlabel("x [m]")
#plt.ylabel("y [m]")
#plt.title("Your measured XY positions")
#plt.show()


### Mess around...
#i=5
s    = 0.03774577043215102
f    = 2.4920861411158968
u0   = -0.03593004847648046
v0   = -0.017834175469668728
dX   = 2.2
dY   = 3.0
dZ   = 1.8160867947267616
m_x  = 2.1676396380246783
m_y  = 1.4742667205332163
gamma= 2.4574827387577995
r0   = 0.0
r1   = 0.0
r2   = 0.0
#LHS = s*numpy.array([[proj_xy[i,0]],[proj_xy[i,1]],[1]]).T
#K = numpy.array([[f*m_x, gamma, u0, 0], [0, f*m_y, v0, 0], [0, 0, 1, 0]])
#Rt = numpy.zeros((4, 4))
#Rt[0:4, 0:4] = numpy.eye(4)
#dX = 0; dY = 0; dZ = 1.84
#Rt[:, -1] = numpy.array([dX, dY, dZ, 1])
#RHS = numpy.dot(numpy.dot(K, Rt), numpy.array([calibPointsXYZ[i,0], calibPointsXYZ[i,1], calibPointsXYZ[i,2], 1]).T)


### Set up least squared problem with reasonable initial guess
# 
# s*[x,y] = K.[X,Y,Z,1]
#
#     [ f*m_x gamma    u_0    0 ]
# K = [   0   f*m_y    v_0    0 ] 
#     [   0      0      1     0 ]

x0 = [s, f, u0, v0, dX, dY, dZ, m_x, m_y, gamma, r0, r1, r2]


def rotationMatrix(r):
    # https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
    # its length is the rotation angle
    rotationAngleDeg = numpy.linalg.norm(r)

    if rotationAngleDeg > rotationAngleDegThreshold:
        # its direction is the rotation axis.
        rotationAxis = r / rotationAngleDeg

        # positive angle is clockwise
        K = numpy.array([[       0,         -rotationAxis[2],  rotationAxis[1]],
                            [ rotationAxis[2],        0,         -rotationAxis[0]],
                            [-rotationAxis[1],  rotationAxis[0],        0       ]])

        # Note the numpy.dot is very important.
        R = numpy.eye(3) + (numpy.sin(numpy.deg2rad(rotationAngleDeg)) * K) + \
            ((1.0 - numpy.cos(numpy.deg2rad(rotationAngleDeg))) * numpy.dot(K, K))

        tmp = numpy.eye(4)
        tmp[0:3, 0:3] = R
    else:
        R = numpy.eye(3)
    return R

global j
j = 0 

def optimiseMe(x):
    global j
    j += 1
    s, f, u0, v0, dX, dY, dZ, m_x, m_y, gamma, r0, r1, r2 = x
    
    Rt = numpy.zeros((4, 4))
    
    #Rt[0:4, 0:4] = numpy.eye(4) # no rotation for now
    R = rotationMatrix(numpy.array([r0, r1, r2]))
    Rt[0:3, 0:3] = R
    #print(R)
    Rt[:, -1] = numpy.array([dX, dY, dZ, 1])
    K = numpy.array([[f*m_x, gamma, u0, 0], [0, f*m_y, v0, 0], [0, 0, 1, 0]])
    
    totalError = 0
    for i in range(9):
        RHS = numpy.dot(numpy.dot(K, Rt), numpy.array([calibPointsXYZ[i,0], calibPointsXYZ[i,1], calibPointsXYZ[i,2], 1]).T)/s
        totalError += numpy.square(RHS[0:2] - proj_xy[i]).sum()
    
    if j%100 == 0: print(f"{dX} {dY} {dZ}: {totalError}")
    return totalError 




#output = scipy.optimize.least_squares(optimiseMe, x0, verbose=True, max_nfev=100000000)
output = scipy.optimize.minimize(optimiseMe, x0, method='Powell', options={'disp': True})
#if output["success"]:
s, f, u0, v0, dX, dY, dZ, m_x, m_y, gamma, r0, r1, r2 = output["x"]
print(f"s    : {s    }")
print(f"f    : {f    }")
print(f"u0   : {u0   }")
print(f"v0   : {v0   }")
print(f"dX   : {dX   }")
print(f"dY   : {dY   }")
print(f"dZ   : {dZ   }")
print(f"m_x  : {m_x  }")
print(f"m_y  : {m_y  }")
print(f"gamma: {gamma}")
print(f"r0   : {r0   }")
print(f"r1   : {r1   }")
print(f"r2   : {r2   }")



### Show residuals in mm of computed optimum
print("\n\nFinal Quality check!!\n\n")

Rt = numpy.zeros((4, 4))

#Rt[0:4, 0:4] = numpy.eye(4) # no rotation for now
R = rotationMatrix(numpy.array([r0, r1, r2]))
Rt[0:3, 0:3] = R
#print(R)
Rt[:, -1] = numpy.array([dX, dY, dZ, 1])
K = numpy.array([[f*m_x, gamma, u0, 0], [0, f*m_y, v0, 0], [0, 0, 1, 0]])

totalError = 0
for i in range(9):
    RHS = numpy.dot(numpy.dot(K, Rt), numpy.array([calibPointsXYZ[i,0], calibPointsXYZ[i,1], calibPointsXYZ[i,2], 1]).T)/s
    print(f"Input pixels: {proj_xy[i]}, output match: {RHS[0:2]}")
    
    
    
    
    
# Very rough AR test
import sys
#import ogl_viewer.viewer as gl
import pyzed.sl as sl
import numpy as np
import progressbar
import tifffile
import scipy.ndimage
import matplotlib.pyplot as plt
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
    #for n in progressbar.progressbar(range(medianFrames)):
    for n in range(medianFrames):
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
