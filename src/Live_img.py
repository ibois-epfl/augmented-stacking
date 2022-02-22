import numpy as np
import sys
if sys.version_info[0] == 2:  # the tkinter library changed it's name from Python 2 to 3.
    import Tkinter
    tkinter = Tkinter #I decided to use a library reference to avoid potential naming conflicts with people's programs.
else:
    import tkinter

from PIL import Image, ImageTk
import Calibration_functions
import cv2
import pyzed.sl as sl
import tifffile

def Draw_ROI(img):
    h,w = img.shape[:2]
    Zone = np.copy(img)
    ROI = Calibration_functions.ROI
    Zone[ROI[0].start,ROI[1]] = [255,0,0]
    Zone[ROI[0].stop,ROI[1]] = [255,0,0]
    Zone[ROI[0],ROI[1].start] = [255,0,0]
    Zone[ROI[0],ROI[1].stop] = [255,0,0]
    return Zone


def live_stream():

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

    ###########################################################################################################################################
    ### Setting point cloud params 
    ###########################################################################################################################################

    point_cloud = sl.Mat(zed.get_camera_information().camera_resolution.width, 
                            zed.get_camera_information().camera_resolution.height,
                            sl.MAT_TYPE.F32_C4,
                            sl.MEM.CPU)

    print("Loading previous Background.tiff")
    background = tifffile.imread('Background.tiff')
    backgroundROI = background[Calibration_functions.ROI]
    print(f"I loaded a background image with shape: {background.shape}")

    imXYZ = Calibration_functions.get_image(zed, point_cloud, medianFrames=3, components=[0,1,2])

    imXYZroi = imXYZ[Calibration_functions.ROI[0],
                    Calibration_functions.ROI[1]]

    P = np.load("3D_2D/3D_2D_matrix.npy")

    zMin = 0
    zMax = 0.2

    npArrayIm = np.zeros((1080,1920,3), dtype=np.uint8)
    for DX in range(imXYZroi.shape[0]):
        for DY in range(imXYZroi.shape[1]):
            X = imXYZroi[DX, DY, 0]
            Y = imXYZroi[DX, DY, 1]
            Z = imXYZroi[DX, DY, 2]
            # convert XYZ to pixels
            pixels = np.dot(P, np.array([X, Y, Z, 1]).T)
            
            #print(pixels)
            # checks for array limits!?
            Z -=  backgroundROI[DX, DY]
                    
            #if Z < zMin: Z = zMin
            #if Z > zMax: Z = zMax
            #print(Z)
            outputValue = 255*(Z - zMin)/zMax
            
            if np.isfinite(pixels[0]) and np.isfinite(pixels[1]):
                x = int(np.round(pixels[0]))
                y = int(np.round(pixels[1]))
                if x >= 0 and x < npArrayIm.shape[0]:
                    if y >= 0 and y < npArrayIm.shape[1]: 
                        #print(f"setting {x}-{y} to {outputValue}")
                        npArrayIm[x-4:x+3,
                                    y-4:y+3,1] = int(outputValue)
    zed.close()
    return npArrayIm
    

def main():
    root = tkinter.Tk()
    w, h = root.winfo_screenwidth(), root.winfo_screenheight()
    root.geometry("%dx%d+0+0" % (w, h))
    root.attributes('-fullscreen', True)
    root.bind('<Escape>', lambda e: root.quit())
    lmain = tkinter.Label(root)
    lmain.pack()

    def show_frame():
        frame = live_stream()
        frame = Draw_ROI(frame)
        imgtk = ImageTk.PhotoImage(image=Image.fromarray(frame, mode="RGB"))
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)
        lmain.after(10, show_frame)
    show_frame()
    root.mainloop()

main()