# augmented-stacking

![stacking_image](./img/augmented_stones_finalShot_v1.png)

Development code for an dry-stone and -timber stacking technique in AR. Developed within the research of Cluster ENAC 2022 (EPFL). And thanks to the work of Andrea Settimi (IBOIS, EPFL), Edward Andò (Imaging Analysis Center, EPFL), Qianqing Wang (EESD, EPFL) and the coordination of Julien Gamerro (IBOIS, EPFL)..


---

## 2022.02.03 - Installation of set-up + point cloud grabber

<img src="./img/grabber.png" width="400" height="400">

Installation of preliminary set-up one (rgbd + projector + metal frame + cables)

Also tested the pcd grabber and viewer with ZED sdk.


---

## 2022.02.04 - First draft of calibration between rgbd camera and projector

<img src="./img/IMG_20220204_101414.jpg" width="400" height="400">

<img src="./img/IMG_20220204_122843.jpg" width="400" height="400">

First functioning draft of the calibration process between the projector and the rgbd camera is done. 

To do list for the next steps:
* Try to find better cable?
* Nicer calibration object
* Measure a number of positions on a given (and recorded!! grid)
* Find out what this mysterious 4th field is from the output
* Get head around x=PX

---
## 2022.02.08 - Meeting for integration stacking algorithm

- QQ meeting stacking alogrithm

- the mesh has to be watertight

- guidance system: 
-- seames/edges
-- contour (to start off we are going just to output a single point from the 3D space, converted e.g., in a circle in 2D)
-- label to say which one to pick
-- deviation betwwen physical and virtual model to correct the physical rock placement to be as close as possible to the virtual model---> this would be just another piece of code different from the stacking algorithm.

TODO list:
- QQ: will start working on the comparison code
- ANDREA:
--a) point cloud of the stone and real stone 
--b) point cloud of the stone within the landscape

for instructing the workers we will give a colored landscape (red or gree) to guide the correct altitude of the stone.

- axe alignement physical and virtual: we are just going to place the origin in the physical world and won't move it.

- library of stones: QQ needs to have meshed simplified. The library will contain two versions [high res and low res (with spherical armonics simplification).

The format of the library is .ply, .obj and pcd format.

-- EESD will do the downsampled spherical armonics

- PIPELINE OF ACQUIRED POINT CLOUD TO 3D STACKING ALGORITHM: we will export the captured point cloud with origin as center of the image because QQ's center is the middle of the rectangle ground. 

-- ANDREA: do the datflow of chulckboard

To sum up the dataflow so far:

![dataflow_v1](./img/stacking_alg_dataf.drawio.png)


TODO LIST:
-- ANDREA:
--- do the dataflow of the pitcure taken from meeting 
--- scan stone and give physical and model to QIANQING
--- 3D TO 2D pointcloud pipeline with QQ's virtual cordinates

-- QIANQING:
--- preliminary test on deviation feedback between acquired point cloud and computed 3D model

---
## 2022.02.15 - Calibration Enhencement and fullscreen projections

<img src="./img/Calibration.jpg" width="400" height="400">

<img src="./img/Calibration_object.jpg" width="400" height="400">

New Disk used for Calibration.

A Full screen image was cast on the object, representing a depth map of the objects.

To do list for the next steps:
* Find out if the distortion of the projector gives us a better precision. (< 74 func error) - DONE, YES
* Try making a live image - DONE FOR THE CALIB
* Cleaning up the code - YES SORT OF
* Cover the Object with paint, make it less reflective - SANDED,  BUT WE MIGHT PAINT IT (DOESN CHANGE MUCH)
* Make new calibration target super-easy for the Zed to detect - DONE
* Optional but very nice: plot ROI on the AR system to see edges of vision - DONE
* Two complete calibrations (save data without overwriting!) with more points (4x5 grid) with and without distortion correction and compare residuals. If residual higher with projector “distortion correction” → we should implement distortion in the projection function. If the same → Error is coming from somewhere else. If lower → that’s a pretty good calibration! - GOOD DONE
* Implement a loop to read depth cam and plot either greylevels or objects above a threshold on projector - ONLY DONE FOR THE CALIB
* With the above tool, check the calibration in ROI limits and different Z-levels - LIMITED TO LOW LEVELS (THAT'S OK FOR WHAT WE HAVE TO DO FOR NOW)


## 2022.02.22 - Live Capture and Distortion Test


Live_img file has been created to live display the objects added to the scene and project them in green.

For Calibration now use show_calib_img code too display in full screen the calibration image.

The Calibration Object has been fixed, but still got some reflection problem with sunlight reflexion, see image bellow:

<img src="./img/Calibration_object_reflection_issue.jpg"  height="400">

<img src="./img/Calibration_object_reflection_isuue_computer.jpg" height="400">

It appears that with a Distortion Correction of -30, we got a lower residual. Now it is at arround 28 against 74 without. So we need to use distortion correction.

<img src="./img/Residual_with_correction.png" height="400">


It appears the Calibration is not that precise in other Z levels, need to see why...
This step need to be done with better lighting conditions (At night for example) 



# 2022-03-02: Integration of stacking algorithm

<img src="./img/IMG_20220302_152714.jpg" width="400" height="400">

Session to start including all software pieces into one common CLI application:
- integration of dataset fetching service to download stones from cloud
- integration of stacking algorithm
- partial integration of devience map

TODO: (Andrea) write code to capture pointcloud with origin in the center
TODD: (Qianqing) adjust the sensitivity of the deviance map coloring
TODO: (Qianqing) for deviance map conider only local area (e.g. sphere around the pointcloud)
TODO: (Andrea) wrap everything in a wrap loop
