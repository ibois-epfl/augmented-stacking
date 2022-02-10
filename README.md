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
