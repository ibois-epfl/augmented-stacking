## This readme is a tutorial for using the calib.py function.

If you want the script to work without any thing to be changed, you should use the following tree, otherwise you will have to change the path in the different python scripts.



1. Generation of the Calibration grid

In order to be able to calibrate the camera, you will first need to generate the calibration image.
To do so, use the following script:

```bash
python calib_img_generator.py -x NB_OF_POINTS_IN_X_DIRECTION -y NB_POINTS_IN_Y_DIRECTION -w WIDTH_IN_PIXL_OF_THE_LINES
```

where:
-  NB_POINTS_IN_X_DIRECTION, has to be replaced by the number of points you want in x direction.
-  NB_POINTS_IN_Y_DIRECTION, is the number of points you want in y direction
-  WIDTH_IN_PIXL_OF_THE_LINES, is the with in pixels of the line used to create the grid.

This script will return two file:

-  calibration_grid.png, which is an image of the calibration grid, with the background in black and the grid in green.
-  Pixels_pts.npy, which is the table of the x,y pixel coordinates of each line intersection, representing the points which wil be used for calibration

2. Full size displaying of the calibration image

Once the Calibration image has been generated, you can display it in full screen using following script:

```bash
python show_calib_image.py
```

This script need the calibration_grid.png image to be in the same folder.
The escape button should allow you to close the fullscreen image.


3. Projector distortion correction

Use the parameters of the projector to correct the distortion if needed, previous calibration experiments have shown that this correction is usefull to enhence the precision.


4. Calibration process

Now you will be able to launch the calib.py script with following parameters:

-  NB_CALIB_PTS, it is the number of calibration points you are going to use, this should correspond to the number of intersection on your calibration grid.
		NB_CALIB_PTS = NB_POINTS_IN_X_DIRECTION * NB_POINTS_IN_Y_DRECTION (See 1.) 
-  CALIB_Z_THRESHOLD, corresponds to the height in meters of the calibration disk used. In our case you can put a value of 0.15m 
-  RADIUS_THRESHOLD_PX, corresponds to the threshold used to select the acceptable disks found in the image. The default value is fine.
-  STARTING_POINT, this is something which can be used if for some reason the calibration process had to be stopped and you want to resume where it stopped, just put the number of the intersection where you stopped.
-  SAVING_PATH, is the path where you want each points to be saved. By default the points are saved in the Calib_Pts/ folder.

Now you can execute the following script:

```bash
python calib -cpts NB_CALIB_PTS [-zT CALIB_Z_THRESHOLD] [-rT RADIUS_THRESHOLD_PX] [-s STARTING_POINT] [-sp SAVING_PATH]
```

values in [] can be ignored for the first launch.

Now you will have to follow the instruction in the bash. 

Which are as followes:

4.1 Background depth acquisition

The first step is the acquisition of the background, make sure the scene is empty bacause we are now acquiring the background which will be taken as the reference for all next measures.
It is kind of our 0 value of all next depth aquisition.

If it the first time you run the calibration for this particular scene, you should make sure you have no file named: Background.tiff in the folder.
Otherwise this will be the reference.

Make sure nothing crosses the scene during this process.

4.2 Point acquisition

This step is where we will put the calibration disk into the scene.
There is an order which has to be followed during the acquisition of the different images.
You start on the top left and go from left to right, starting at the left at each new row. Make sure the order is right.

For each point, try to align the middle of the disk with the crossing of the lines.
Once the image is taken the algorithm will try to detect the disk, if it doesn't for any reason, the terminal will tell you to put the disk on the same position you are working on.
Just restart the process, and make sure no other object appeares in the scene during the acquisition.

Once the disk has been correctly detected, two files will be saved in the SAVING_PATH (Calib_pts/ by default):

-  Image_position_1.np.npy
-  Image_position_Z_1.tiff
 
4.3 The gathering of all points

Once the process has ended another file should be generated in the folder where the calib script is located.

-  calib_points_XYZ.npy

This file corresponds to all the XYZ coordinates of the points you just acquired during the calibration process.

## Next step - The 2D-3D transfer matrix

You will need following files:

- calib_points_XYZ.npy
- Pixels_pts.npy


