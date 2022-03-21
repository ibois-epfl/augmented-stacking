"""
2022-02-14  DR -- Cleaning up calibAnalysis.py

This function is giving us the matrix K, making the link between the xy pixels and the physical 3D space XYZ.

Args: 
    - calibration points: coordinates XYZ in meters, saved in a np file. 
    - grid pixel position: pixel coordinates xy, saved in Pixel_pts.npy
Return:
    - K matrix
Note:
  xy = pixels
  XY = physical space (meters)
"""

from lib2to3.pgen2.token import NUMBER
from platform import java_ver
import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize
import json
import argparse

ROI = [slice(200, 500), slice(400, 900)]


GRAPH = False

## Parameter for the rotationmatrix function
rotationAngleDegThreshold = 0.00001

def rotationMatrix(r):
    # https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
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

    """
    ## for printing purposes during optimisation process
    global j
    j += 1

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
    
    if j%100 == 0: print(f"{dX} {dY} {dZ}: {np.sqrt(totalError)}")
    return np.sqrt(totalError) 


def main(PROJECTOR_PIXEL_PTS_PATH,CALIB_PTS_XYZ,SAVE_PATH,number_of_calibration_pts):
    global NUMBER_OF_CALIBRATION_PTS 
    NUMBER_OF_CALIBRATION_PTS = number_of_calibration_pts
    ###########################################################################################################################################
    ### Loading Calibration data
    ###########################################################################################################################################
    
    ### Load projector positions in px
    proj_xy = np.load(PROJECTOR_PIXEL_PTS_PATH)
    print(proj_xy)

    ### Load CORRESPONDING!!! 3D positions of calibration target
    # calib_points_XYZ = []
    # for point in range(1,NUMBER_OF_CALIBRATION_PTS+1):
    #     calib_points_XYZ.append(np.load(f"calib_pts/Image_position_{point}.np.npy"))
    # calib_pointsXYZ = np.array(calib_points_XYZ)
    # np.save("calib_points_XYZ.npy",calibPointsXYZ)

    calib_points_XYZ = np.load(CALIB_PTS_XYZ)
    print(calib_points_XYZ)

    ###########################################################################################################################################
    ### Finding out the parameter of the K matrix
    ###########################################################################################################################################

    # s*[x,y,1] = K.R(r0,r1,r2).[X,Y,Z,1]
    #
    #     [ f*m_x gamma    u_0    0 ]
    # K = [   0   f*m_y    v_0    0 ] 
    #     [   0      0      1     0 ]
    #

    # Initialisation
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
                                     method='Powell',
                                     options={'disp': True,
                                              'xtol': 0.00000001,
                                              'ftol': 0.0000001,
                                              'maxiter': 1000})


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

    # Json file saving as a dictionary
    K_dict = {"s": s , "f": f , "u0":u0 , "v0":v0 , "dX":dX , "dY":dY , "dZ":dZ , "m_x":m_x , "m_y":m_y , "gamma":gamma , "r0":r0 , "r1":r1 , "r2":r2 }
    K_matrix = open(SAVE_PATH+".json", "w")
    json.dump(K_dict, K_matrix)
    K_matrix.close()

    # numpy 3*4 matrix saving
    P_matrix = np.dot(K,Rt)/s
    np.save(SAVE_PATH+".npy",P_matrix)
    print("Matrix has succesfully been saved")


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description="Get the Calibration Points",
                                   formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        '-cpts', '--nbCalibPts',
        help="Number of Calibration points.",
        type=int,
        default=9
    )
    parser.add_argument(
        '-cP', '--calibPoints',
        help="Path of the calibration points npy file, with all files together.",
        type=str,
        default=""
    )
    parser.add_argument(
        '-pP', '--pixelPoints',
        help="Path of the pixel points position on the grid, npy file generated with the calibration image.",
        type=str,
        default=""
    )
    parser.add_argument(
        '-o', '--outputPath',
        help="Path where the 2D_3D matrix will be saved.",
        type=str,
        default=""
    )
    args = parser.parse_args()

    
    main(args.pixelPoints,args.calibPoints,args.outputPath,args.nbCalibPts)

