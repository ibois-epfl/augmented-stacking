import numpy as np
import tifffile
import sys
if sys.version_info[0] == 2:  # the tkinter library changed it's name from Python 2 to 3.
    import Tkinter
    tkinter = Tkinter #I decided to use a library reference to avoid potential naming conflicts with people's programs.
else:
    import tkinter
from PIL import Image, ImageTk
import yaml
import Calibration_functions
sys.path.append('/usr/local/lib/python3.8/dist-packages')
import cv2



## Parameter for the rotationmatrix function
rotationAngleDegThreshold = 0.00001

def rotationMatrix(r):
    """
    Simple 3D Matrix rotation function, obtained from following sources:
    https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula

    Args: 
        -r: a rotation vector, with rotation value in x, y and z direction.
    """
    # its length is the rotation angle
    rotationAngleDeg = np.linalg.norm(r)

    if rotationAngleDeg > rotationAngleDegThreshold:
        # its direction is the rotation axis.
        rotationAxis = r / rotationAngleDeg

        # positive angle is clockwise
        K = np.array([[       0,         -rotationAxis[2],  rotationAxis[1]],
                            [ rotationAxis[2],        0,         -rotationAxis[0]],
                            [-rotationAxis[1],  rotationAxis[0],        0       ]])

        # Note the np.dot is very important.
        R = np.eye(3) + (np.sin(np.deg2rad(rotationAngleDeg)) * K) + \
            ((1.0 - np.cos(np.deg2rad(rotationAngleDeg))) * np.dot(K, K))

        tmp = np.eye(4)
        tmp[0:3, 0:3] = R
    else:
        R = np.eye(3)
    return R


def get_3D_2D_matrix(YAML_PATH):
    """
    This function opens the Calibration Yaml File, reads the information about the 3D_2D_matrix and return it as a numpy array

    Args:
        - YAML_PATH: Path of the yaml calibration file, containing the 3D_2D_Matrix information.
    """

    # Opening YAML file
    with open(YAML_PATH) as yaml_file:
        data = yaml.load(yaml_file,Loader=yaml.FullLoader)
    print(data)
    Matrix = data["3D_2D_Matrix"]
    s, f, u0, v0, dX, dY, dZ, m_x, m_y, gamma, r0, r1, r2 = Matrix["s"],Matrix["f"],Matrix["u0"],Matrix["v0"],Matrix["dX"],Matrix["dY"],Matrix["dZ"],Matrix["m_x"],Matrix["m_y"],Matrix["gamma"],Matrix["r0"], Matrix["r1"],Matrix["r2"]
    Rt = np.zeros((4, 4))
    R = rotationMatrix(np.array([r0, r1, r2]))
    Rt[0:3, 0:3] = R
    Rt[:, -1] = np.array([dX, dY, dZ, 1])
    K = np.array([[f*m_x, gamma, u0, 0], [0, f*m_y, v0, 0], [0, 0, 1, 0]])
    From_3D_2D_matrix = np.dot(K,Rt)/s
    
    return From_3D_2D_matrix



###########################################################################################################################################
### Setting ZED params
###########################################################################################################################################
import pyzed.sl as sl

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

print("Loading previous Background.tiff")
background = tifffile.imread('Background.tiff')
print(f"I loaded a background image with shape: {background.shape}")

imXYZ = Calibration_functions.get_image(zed, point_cloud, medianFrames=1, components=[0,1,2])
print(imXYZ.shape)


# threshold out the points
zMask = imXYZ[Calibration_functions.ROI[0],Calibration_functions.ROI[1],2].copy()-background[Calibration_functions.ROI] > 0.05
#print(points.sum(), points.mean())

#import matplotlib.pyplot as plt
#plt.imshow(zMask)
#plt.show()

pointsXYZ = imXYZ[Calibration_functions.ROI[0],
                  Calibration_functions.ROI[1],:][zMask][::1]

P = get_3D_2D_matrix("/opt/augmented-stacking/CLI_AS/calib/utils/calibration_info.yaml")

#pixels = numpy.zeros([pointsXYZ.shape[0],2])
numpyArrayIm = np.zeros((1080,1920), dtype=bool)
for X, Y, Z in pointsXYZ:
    # convert XYZ to pixels
    # print(f"X:{X} - Y:{Y} - Z:{Z}")
    pixels = np.dot(P, np.array([[X], [Y], [Z],[1]]))
    # print(pixels)
    # checks for array limits!?
    if pixels[0]<1080 and pixels[1]<1920:
        numpyArrayIm[int(pixels[0]),
                    int(pixels[1])] = 1

zed.close()

import scipy.ndimage
numpyArrayIm = scipy.ndimage.binary_dilation(numpyArrayIm, iterations=5)

outputIm = np.zeros((numpyArrayIm.shape[0], numpyArrayIm.shape[1], 3), dtype='<u1')
outputIm[:,:,1] = numpyArrayIm.astype('<u1')*255

# root = tkinter.Tk()
# w, h = root.winfo_screenwidth(), root.winfo_screenheight()
# #root.overrideredirect(1)
# root.geometry("%dx%d+0+0" % (w, h))
# root.attributes('-fullscreen', True)
# canvas = tkinter.Canvas(root,width=w,height=h, highlightthickness=0)
# root.bind("<Escape>", lambda e: root.destroy())
# canvas.pack()
# canvas.focus_set()       
# image = ImageTk.PhotoImage(image=Image.fromarray(outputIm, mode="RGB"))
# imagesprite = canvas.create_image(w/2,h/2,image=image)
# root.mainloop()

cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("window",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
cv2.imshow("window", outputIm)
cv2.waitKey(0)
