import numpy as np
from calibration_functions import get_3D_2D_matrix

# for i in range(9):
#     image_point_path = f"utils/points/coordinates/Image_position_{i+1}.npy"
#     camera_point_path = f"utils/points/coordinates/Camera_px_position_{i+1}.npy"
#     point_cam = np.load(camera_point_path)
#     point = np.load(image_point_path)
#     print(point)
#     print(point_cam)

All_3D = np.load('utils/points/calib_3D_points.npy')
print(All_3D)

All_Cam_2D = np.load("utils/grid/camera_2D_pixel_coordinates.npy")
print(All_Cam_2D)

All_proj_2D = np.load("utils/grid/2D_pixel_coordinates.npy")
print(All_proj_2D)

_3D_path = "utils/calibration_info.yaml"

P = get_3D_2D_matrix(_3D_path)

print(P)
X,Y,Z = 1.03,-0.22,-2.05
# test = np.dot(P,np.array([[All_3D[0][0]],[All_3D[0][1]],[All_3D[0][2]],[1]]))
test = np.dot(P,np.array([[X],[Y],[Z],[1]]))
print(test)

