import numpy as np
import sys
if sys.version_info[0] == 2:  # the tkinter library changed it's name from Python 2 to 3.
    import Tkinter
    tkinter = Tkinter #I decided to use a library reference to avoid potential naming conflicts with people's programs.
else:
    import tkinter

from PIL import Image
from PIL import ImageTk
import Calibration_functions
import pyzed.sl as sl
import tifffile

P = np.load("3D_2D/Distortion_correction_-30_3D_2D_matrix.npy")
print(P)
def Draw_ROI(imXYZ,img):
    
    # Most of the time the ROI is outside the preojectors range, so we will not be able to draw the roi correctly...
    Zone = np.copy(img)
    h,w = img.shape[:2]
    ROI = Calibration_functions.ROI
    P11 = ROI[1].start
    P12 = ROI[1].stop
    P21 = ROI[0].start
    P22 = ROI[0].stop
    
    Y1,X1,Z1 = imXYZ[P21,P11]
    Y2,X2,Z2 = imXYZ[P22,P11]
    Y3,X3,Z3 = imXYZ[P21,P12]
    
    Pt1 = np.array([[X1],[Y1],[Z1],[1]])
    Pt2 = np.array([[X2],[Y2],[Z2],[1]])
    Pt3 = np.array([[X3],[Y3],[Z3],[1]])
    
    Pix1 = np.dot(P,Pt1)
    Pix2 = np.dot(P,Pt2)
    Pix3 = np.dot(P,Pt3)
    
    if not (int(Pix1[1]) in range(0,h)):
        Pix1[1] = 0
    if not (int(Pix2[1]) in range(0,h)):
        Pix2[1] = h-1
    if not (int(Pix1[0]) in range(0,w)):
        Pix1[0] = 0
    if not (int(Pix3[0]) in range(0,w)):
        Pix3[0] = w-1
    print(Pix1[1],Pix1[0],Pix2[1],Pix3[0])
    
    Zone[int(Pix1[1]):int(Pix2[1]),int(Pix1[0])] = [255,0,0]
    Zone[int(Pix1[1]):int(Pix2[1]),int(Pix3[0])] = [255,0,0]
    Zone[int(Pix1[1]),int(Pix1[0]):int(Pix3[0])] = [255,0,0]
    Zone[int(Pix2[1]),int(Pix1[0]):int(Pix3[0])] = [255,0,0]
    
    
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
    
    
# threshold out the points
    zMask = imXYZ[Calibration_functions.ROI[0],Calibration_functions.ROI[1],2].copy()-background[Calibration_functions.ROI] > 0.1
    
    imXYZroi = imXYZ[Calibration_functions.ROI[0],
                    Calibration_functions.ROI[1]]


    pointsXYZ = imXYZroi[zMask][::1]

    #pixels = numpy.zeros([pointsXYZ.shape[0],2])
    numpyArrayIm = np.zeros((1080,1920,3), dtype=np.uint8)
        
    for X, Y, Z in pointsXYZ:
        # convert XYZ to pixels
        pixels = np.dot(P, np.array([[X], [Y], [Z],[1]]))
        if pixels[0]<1080 and pixels[1]<1920:
            numpyArrayIm[int(pixels[0]),
                    int(pixels[1]),1] = 255
    
    numpyArrayIm = Draw_ROI(imXYZ,numpyArrayIm)
    zed.close()    
    
    return numpyArrayIm
    

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
        imgtk = ImageTk.PhotoImage(image=Image.fromarray(frame, mode="RGB"))
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)
        lmain.after(5, show_frame)
    show_frame()
    root.mainloop()

main()
