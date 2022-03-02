## This readme file will show you how to use the 3D_2D_transform.py script

For this script all you will need is the following files:

-  calib_points_XYZ.npy
-  Pixels_pts.npy

See readme_calib.md for more information.


The following arguments have to be used in the 3D_2D_transform.py script:

-  NB_CALIB_PTS, the number of calibration points used during calibration
-  PATH_CALIB_POINTS, the path of the calibration points XYZ coordinates, you measured with the camera using the calib.py script.
-  PATH_PIXEL_POINTS, the path of the xy pixel coordinates of the line intersections of the calibration grid.
-  OUTPUT_PATH, the path where the 3D_2D transfer matrix will be saved.

Having those, you just need to execute the following script:

```bash
python 3D_2D_transform.py -cpts NB_CALIB_PTS -cP PATH_CALIB_POINTS -pP PATH_PIXEL_POINTS -o OUTPUT_PATH
```

This script will generate following file:
-  3D_2D_matrix.npy
-  3D_2D_matrix.json


Having those you can simply make the link between the 3D space and the project 2D space.

By using the following python code, you can get from the XYZ space to the xy space and vise versa:

```bash
import numpy as np

P=np.load("3D_2D_matrix.npy")
P_inv = np.linalg.inv(P)
## From 3D to 2D
XYZ1 =  np.array([X, Y, Z, 1]).T
xy1 = np.dot(P,XYZ1)
xy = xy1[0:2]

## From 2D to 3D
xy1 = np.array([x,y,1]).T
XYZ1 = np.dot(xy1,P_inv)
XYZ = XYZ1[0:3]

```


