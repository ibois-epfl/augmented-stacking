
import sys
import pyzed.sl as sl
import numpy as np
import tifffile
import scipy.ndimage
import matplotlib.pyplot as plt
import os.path
import os
from tqdm import tqdm
import skimage.measure
from PIL import Image
from PIL import ImageTk
import yaml
import threading
if sys.version_info[0] == 2:  # the tkinter library changed it's name from Python 2 to 3.
    import Tkinter as tk
else:
    import tkinter as tk

############################################################################################################################################
############################################### Function used for 3D-2D matrix estimation ##################################################
############################################################################################################################################

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

def optimise_me(x,calib_points_XYZ,proj_xy):
    """
    This is the function we want to optimize. It corresponds to the following matrix equation:

    s*[x,y,1] = K.Rt(r0,r1,r2,dX,dY,dZ).[X,Y,Z,1]

    with:

        [ f*m_x gamma    u_0    0 ]
    K = [   0   f*m_y    v_0    0 ] 
        [   0      0      1     0 ]

    Args:
        - x: The initial guess of the parameters
        - calib_points_XYZ: The 3D coordinates of the points measured during calibration, in a numpy array (n,3), with n the number of calibration points.
        - proj_xy: The 2D coordinates, obtained during the calibration grid generation, in a numpy array (n,2)
        - NUMBER_OF_CALIBRATION_PTS: the number of Calibration 

    """
    ## for printing purposes during optimisation process
    global j
    j += 1
    NUMBER_OF_CALIBRATION_PTS = calib_points_XYZ.shape[0]

    ## Initialisation
    s, f, u0, v0, dX, dY, dZ, m_x, m_y, gamma, r0, r1, r2 = x
    
    # Rotation matrix
    R = rotationMatrix(np.array([r0, r1, r2]))

    # Rotation and translation
    Rt = np.zeros((4, 4))
    Rt[0:3, 0:3] = R
    Rt[:, -1] = np.array([dX, dY, dZ, 1])

    # K matrix
    K = np.array([[f*m_x, gamma, u0, 0], [0, f*m_y, v0, 0], [0, 0, 1, 0]])
    
    totalError = 0
    for i in range(NUMBER_OF_CALIBRATION_PTS):
        # Right Hand Side, See equation above
        XYZ1 = np.array([calib_points_XYZ[i,0], calib_points_XYZ[i,1], calib_points_XYZ[i,2], 1]).T
        RHS = np.dot(np.dot(K, Rt), XYZ1)/s
        totalError += np.square(RHS[0:2] - proj_xy[i]).sum()
    
    if j%1000 == 0: print(f"Error: {np.sqrt(totalError)}")
    return np.sqrt(totalError) 

def calculate_3D_2D_matrix(PROJECTOR_PIXEL_PTS_PATH,CALIB_PTS_XYZ):

    """
    This function is doing the optimization of the optimize_me function.
    It saves the different parameters necessary for the 3D_2D transformation operation, in order to display 3D point cloud with the projector.

    Args:
        - PROJECTOR_PIXEL_PTS: Path to the 2D pixel coordinates obtained in the calibration grid generation.
        - CALIB_PTS_XYZ: Path to the 3D coordinates measured during calibration.
    """
    ### Load projector positions in px
    proj_xy = np.load(PROJECTOR_PIXEL_PTS_PATH)

    calib_points_XYZ = np.load(CALIB_PTS_XYZ)


    # Initialisation
    NUMBER_OF_CALIBRATION_PTS = calib_points_XYZ.shape[0]

    s       = 0.04
    f       = 3.2
    u0       = -0.04
    v0       = -0.02
    dX      = 2.2
    dY      = 3.0
    dZ      = 1.8
    m_x     = 2.2
    m_y     = 1.5
    gamma   = 2.5
    r0      = 0.0
    r1      = 0.0
    r2      = 0.0

    x0 = [s, f, u0, v0, dX, dY, dZ, m_x, m_y, gamma, r0, r1, r2]


    # Optimisation
    global j
    j=0
    output = scipy.optimize.minimize(optimise_me,
                                     x0,
                                     args=(calib_points_XYZ,proj_xy),
                                     method='L-BFGS-B',
                                     options={'disp': True,
                                              'gtol': 0.000000000000001,
                                              'ftol': 0.000000000000001,
                                              'maxiter': 1000000,
                                              'maxcor':10000,
                                              'eps':0.00000000005,
                                              'maxfun':10000000,
                                              'maxls':50000})


    # Results

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

    pause()
    ### Show residuals in mm of computed optimum
    print("\n\nFinal Quality check!!\n\n")

    Rt = np.zeros((4, 4))

    R = rotationMatrix(np.array([r0, r1, r2]))
    Rt[0:3, 0:3] = R
    Rt[:, -1] = np.array([dX, dY, dZ, 1])
    K = np.array([[f*m_x, gamma, u0, 0], [0, f*m_y, v0, 0], [0, 0, 1, 0]])

    for i in range(NUMBER_OF_CALIBRATION_PTS):
        RHS = np.dot(np.dot(K, Rt), np.array([calib_points_XYZ[i,0], calib_points_XYZ[i,1], calib_points_XYZ[i,2], 1]).T)/s
        print(f"Input pixels: {proj_xy[i]}, output match: {RHS[0:2]}")

    # yaml file saving as a dictionary
    K_dict = {"s": float(s) , "f": float(f) , "u0":float(u0) , "v0":float(v0) , "dX":float(dX) , "dY":float(dY) , "dZ":float(dZ) , "m_x":float(m_x) , "m_y":float(m_y) , "gamma":float(gamma) , "r0":float(r0) , "r1":float(r1) , "r2":float(r2) }
    return K_dict

def get_3D_2D_matrix(YAML_PATH):
    """
    This function opens the Calibration Yaml File, reads the information about the 3D_2D_matrix and return it as a numpy array

    Args:
        - YAML_PATH: Path of the yaml calibration file, containing the 3D_2D_Matrix information.
    """

    # Opening YAML file
    with open(YAML_PATH) as yaml_file:
        data = yaml.load(yaml_file,Loader=yaml.FullLoader)
    Matrix = data["3D_2D_Matrix"]
    s, f, u0, v0, dX, dY, dZ, m_x, m_y, gamma, r0, r1, r2 = Matrix["s"],Matrix["f"],Matrix["u0"],Matrix["v0"],Matrix["dX"],Matrix["dY"],Matrix["dZ"],Matrix["m_x"],Matrix["m_y"],Matrix["gamma"],Matrix["r0"], Matrix["r1"],Matrix["r2"]
    Rt = np.zeros((4, 4))
    R = rotationMatrix(np.array([r0, r1, r2]))
    Rt[0:3, 0:3] = R
    Rt[:, -1] = np.array([dX, dY, dZ, 1])
    K = np.array([[f*m_x, gamma, u0, 0], [0, f*m_y, v0, 0], [0, 0, 1, 0]])
    From_3D_2D_matrix = np.dot(K,Rt)/s
    
    return From_3D_2D_matrix


############################################################################################################################################
############################################### Function for the Gen and Display of the calibration grid ###################################
############################################################################################################################################


def display_calibration(CALIB_IMG_PATH):

    """
    This function is displaying an image in full size on your monitor, using Tkinter.
    To escape the full screen just press the escape key of the keyboard.

    Args:
        - CALIB_IMG_PATH: the path of the image displayed, here it is used for the calibration image.
    """

    class App(threading.Thread):

        def __init__(self):
            threading.Thread.__init__(self)
            self.alive = True
            self.start()

        def callback(self):
            self.tk.quit()
        
        def toggle_fullscreen(self, event=None):
            self.state = not self.state  # Just toggling the boolean
            self.tk.attributes("-fullscreen", self.state)
            return "break"

        def end_fullscreen(self, event=None):
            self.state = False
            self.tk.attributes("-fullscreen", False)
            return "break"


        def close(self,event=None):
            self.alive = not self.alive
            if not self.alive :
                self.lmain.configure(compound="center",text="Close Window ?\n Press <Enter> to Confirm.\n Press <q> to exit.",font=("Courier", 44),fg="white",bg="black")
            else:
                self.lmain.configure(text="")

            return "break"

        def down(self,event=None):
            if not self.alive:
                root = self.tk
                root.quit()
                return "break"

        def error_down(self):
            root = self.tk
            root.quit()


        def run(self):
            self.tk = tk.Tk()
            self.tk.attributes('-zoomed', True)  # This just maximizes it so we can see the window. It's nothing to do with fullscreen.
            image = Image.open(CALIB_IMG_PATH)
            self.image = ImageTk.PhotoImage(image=image)
            lmain = tk.Label(self.tk,image=self.image)
            lmain.pack()
            self.lmain = lmain
            self.state = False
            self.tk.bind("<F11>", self.toggle_fullscreen)
            self.tk.bind("<Escape>", self.end_fullscreen)
            self.tk.bind("<q>", self.close)
            self.tk.bind("<Return>",self.down)
            self.tk.protocol("WM_DELETE_WINDOW", self.callback)
            self.tk.mainloop()

            
    
    app = App()
    return app

def draw_grid(save_path_img,save_path_2D_pts,nb_lines_X=3,nb_lines_Y=3,line_width=4):

    """
    This function is generating the grid image and the pixel coordinates of the points.

    Args:
      save_path: path, where the files are saved.
      nb_lines_X: number of lines drawn in X direction, (corresponding to the number of points in X direction)
      nb_lines_Y: number of lines drawn in Y direction, (corresponding to the number of points in Y direction)
      line_width: the width in  pixel of the lines which are drawn.

    Returns:
      An RGB image of the grid used for calibration.
      A numpy file containing the coordinates of the (nb_lines_X * nb_lines_Y) points in pixel.  
    """


    X =[]
    Y =[]
    # Initialize black image
    shape=(1080,1920,3)
    Img = np.zeros(shape,dtype=np.uint8)

    # Calculate space between lines
    X_space = (shape[1] - nb_lines_X*line_width)//(nb_lines_X+1)
    Y_space = (shape[0] - nb_lines_Y*line_width)//(nb_lines_Y+1)

    #Pts coordinate saving 
    Pts=np.zeros((nb_lines_Y*nb_lines_X,2))

    # Draw the lines
    for i in range(1,nb_lines_Y+1):
        Img[i*Y_space-line_width//2:i*Y_space+line_width//2,:,1]=255
        for j in range (1,nb_lines_X+1):
            Pts[(i-1)*(nb_lines_X)+(j-1),0]=j*X_space+line_width//2
            Pts[(i-1)*(nb_lines_X)+(j-1),1]=i*Y_space+line_width//2
    for i in range(1,nb_lines_X+1):
        Img[:,i*X_space-line_width//2:i*X_space+line_width//2,1]=255
    print(Pts)
    np.save(save_path_2D_pts,Pts)
    plt.imsave(save_path_img,Img)
    print(f"A Calibration image of size: {nb_lines_X}x{nb_lines_Y} was generated.\nIt is saved in: {save_path_img}")
   
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
    for n in tqdm(range(medianFrames)):
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,sl.MEM.CPU, zed.get_camera_information().camera_resolution)
            point_cloud_np = point_cloud.get_data()
            stack_of_images.append(point_cloud_np)      
        else:
            print(":(")
            return None
    print("\nThe Scene can now be enterd.\nProcessing images ...")
    stack_of_images = np.array(stack_of_images)
    stack_of_images[not np.isfinite] = np.nan
    median = np.nanmedian(stack_of_images, axis=0)

    return median[:,:,components]

def pause():
    programPause = input("Press the <ENTER> key to continue...")

def get_Disk_Position(imageZoffset, newImageXYZ,ROI,CALIB_Z_THRESHOLD_M,RADIUS_TOLERANCE,RADIUS_PERI_THRESHOLD_PX):

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
      And the Pixel value of the center.
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

            D_position = scipy.ndimage.map_coordinates(newImageXYZ[ROI[0], ROI[1], d], coordsPx,order=1,mode="nearest")[0]
            coordsXYZm.append(D_position)
    elif circlesBool.sum() == 0:
        print("No suitable objects found")
        return None
    else:
        print("More than one suitable object found, need to select best one...")
        return None
    print(f"Found your disk at (x,y,z in meters): {coordsXYZm}")
    return coordsXYZm,centroidPx

## Point Cloud acquisition for Andrea


############################################################################################################################################
#### Setting ZED params
############################################################################################################################################

## Set ZED params
#init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, # HD720 | 1280*720
                            #camera_fps=30, # available framerates: 15, 30, 60 fps
                            #depth_mode=sl.DEPTH_MODE.QUALITY, # posible mods: sl.DEPTH_MODE.PERFORMANCE/.QUALITY/.ULTRA
                            #coordinate_units=sl.UNIT.METER,
                            #coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP, # sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP
                            #sdk_verbose = True, # Enable verbose logging
                            #depth_minimum_distance=0.3, # Enable capture from 30 cm
                            #depth_maximum_distance=3.0 # Enable capture up to 3m
                            #) 

## Open ZED and catch error
#zed = sl.Camera()
#status = zed.open(init)
#if status != sl.ERROR_CODE.SUCCESS:
    #print(repr(status))
    #exit()
#camera_info = zed.get_camera_information()
#print("POP: ZED camera opened, serial number: {0}".format(camera_info.serial_number))


#######################reboo#####################################################################################################################
#### Setting point cloud params 
############################################################################################################################################

#point_cloud = sl.Mat(zed.get_camera_information().camera_resolution.width, 
                        #zed.get_camera_information().camera_resolution.height,
                        #sl.MAT_TYPE.F32_C4,
                        #sl.MEM.CPU)
#XYZ = get_image(zed,point_cloud,components=[0,1,2])

#tifffile.imwrite("A46_point_cloud",XYZ[ROI[0],ROI[1],:])

#zed.close()
