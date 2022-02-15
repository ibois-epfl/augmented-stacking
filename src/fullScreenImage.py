import numpy
import tifffile
import sys
if sys.version_info[0] == 2:  # the tkinter library changed it's name from Python 2 to 3.
    import Tkinter
    tkinter = Tkinter #I decided to use a library reference to avoid potential naming conflicts with people's programs.
else:
    import tkinter
from PIL import Image, ImageTk

import Calibration_functions




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
zMask = imXYZ[Calibration_functions.ROI[0],Calibration_functions.ROI[1],2].copy()-background[Calibration_functions.ROI] > 0.1
#print(points.sum(), points.mean())

#import matplotlib.pyplot as plt
#plt.imshow(zMask)
#plt.show()

pointsXYZ = imXYZ[Calibration_functions.ROI[0],
                  Calibration_functions.ROI[1],:][zMask][::1]

P = numpy.load("3D_2D_matrix.npy")

#pixels = numpy.zeros([pointsXYZ.shape[0],2])
numpyArrayIm = numpy.zeros((1080,1920), dtype=bool)
for X, Y, Z in pointsXYZ:
    # convert XYZ to pixels
    pixels = numpy.dot(P, numpy.array([[X], [Y], [Z],[1]]))
    #print(pixels)
    # checks for array limits!?
    numpyArrayIm[int(numpy.round(pixels[0])),
                 int(numpy.round(pixels[1]))] = 1

zed.close()

import scipy.ndimage
numpyArrayIm = scipy.ndimage.binary_dilation(numpyArrayIm, iterations=5)

outputIm = numpy.zeros((numpyArrayIm.shape[0], numpyArrayIm.shape[1], 3), dtype='<u1')
outputIm[:,:,1] = numpyArrayIm.astype('<u1')*255

print(outputIm[:,:,1].max())

root = tkinter.Tk()
w, h = root.winfo_screenwidth(), root.winfo_screenheight()
#root.overrideredirect(1)
root.geometry("%dx%d+0+0" % (w, h))
root.attributes('-fullscreen', True)
canvas = tkinter.Canvas(root,width=w,height=h, highlightthickness=0)
root.bind("<Escape>", lambda e: root.destroy())
canvas.pack()
canvas.focus_set()       
image = ImageTk.PhotoImage(image=Image.fromarray(outputIm, mode="RGB"))
imagesprite = canvas.create_image(w/2,h/2,image=image)
root.mainloop()
