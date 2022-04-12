# augmented-stacking

![stacking_image](./img/augmented_stones_finalShot_v1.png)

Development code for an dry-stone and -timber stacking technique in AR. 


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


---
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

---
## 2022.03.09 - CLI Draft + Live Depth Capture

Started working on a first draft of a CLI interface to operate during the augmented stacking

<img src="./img/CLIdraft.png" width="400" height="400">


The live capture of the scene as a point cloud is mostly finished. 

<img src="./img/capture.png" width="400" height="400">


In order to avoid to retransform the capture point cloud to the coordinate system of the algorithm working space, we decided to feed the point cloud (once meshed) directly to the stacking algorithm. In doing so the estimated pose will transform the stone directly to the captured scene. In addition, this will allow less deviation in time between the computed model and the real life scene.

<img src="./img/changer.png" width="400" height="400">

---
# 2022.03.12 - Implementing live capture from camera

The point cloud capture is now implemented with a ROI (Region of Interest) corresponding to the building area of the wall.

<img src="./img/capturing_2.png" width="650" height="400">

In order to avoid transformation/calibration processes between the captured coordinate system and the stacking algorithm coordinate system we decided to feed the captured landscape directly to the stacking algorithm. We are now working on meshing correctly (water-tight and high-definition but low density meshes). See here:

<img src="./img/capturing_1.png" width="650" height="400">

**TODO list:**
- Andrea: working on the 3d-2d-3d pipeline for indication / point cloud meshing solutions
- Qianqing: working point cloud meshing solutions + integration of meshed landscape to stacking algorithm

# 2022.04.05 - Implementation of Live Implementing

The image is beeing created and displayed as you can see here:
<p>
    <img src="./img/live_img_1.jpg" width="400">
    <img src="./img/live_img_2.jpg" width="400">
</p>
<img src="./img/live_img_3.jpg" width="400">
We still need to fix a BUG which is occuring when using both the O3d visualizer and the tkinter window for live visualisation.

```bash
X Error of failed request:  BadWindow (invalid Window parameter)
  Major opcode of failed request:  15 (X_QueryTree)
  Resource id in failed request:  0x620000f
  Serial number of failed request:  1858
  Current serial number in output stream:  1858
```

TODO:
- Fixing the bug or changing the Live stream procedure.
- Cleaning up the code
- make pixel biggers for better readability
- config file for augmented stacking algorithm


---
# 2022.04.07 - Changing strategy on augmented instructions

<p>
    <img src="./img/stone_vid_ar_1.gif" width="400">
    <img src="./img/stone_vid_ar_2.gif" width="400">
</p>

The calculation of the deviance map did not provide a data that could be turned into meaningful information to guide the operator to place the stone. 
Here's are the main problems:
- the projected map is hard to follow, especially with a projector in daylight. The distinction between different colors is hard.
- the "occupancy map" by open3d library, as it was used, produced overlaped points since it took in consideration also of points underneath the mesh

So we decided to reorganise the data calculated between the captured point cloud and the computed mesh from the stacking algorithm. Here's the algorithm that will be implemented:

![dataflow_v2](./img/last_stand_stacking_algorithm.png)


TODO LIST:
- implementation of the described code and visualization
- adjustments of config file for stacking algorithm


# 2022.04.11 - Implementation of the code above

The extraction of the upper point cloud of the mesh is as following:

![upper_point_cloud](./img/upper_pcd.png)

The idea is to select 3 keypoints from this point cloud. To do so we use K-mean, which will give us 3 clusters.

Those are the following separations:

<p>
    <img src="./img/cluster_1.png" width="265">
    <img src="./img/cluster_2.png" width="265">
    <img src="./img/cluster_3.png" width="265">
</p>

Those clusters on the meshed rock, will give us 3 bounding boxes, which we can use to subsample the captured point cloud.

For now it seems that the stacking algorithm doesn't allow us to get the right position, so it still need to be adjusted.

This is the final image we get on the screen, with:
- in Green: the convex hull of the projected upper point cloud of the meshed rock.
- in White: the keypoints of the meshed rock, which don't move.
- in Red: the position of the centers of the clustered acquired point clouds.

![Projected visualisation](./img/2d_projection.png)

TODO:

- Visualize the clustered captured points.
- Fix the black points on the convex hull.
- Catch all errors.
- Clean up the Test file.
- See why there is only two points visible, overlapping ?
