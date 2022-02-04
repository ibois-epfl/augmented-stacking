# augmented-stacking

![stacking_image](./img/augmented_stones_finalShot_v1.png)

Development code for an dry-stone and -timber stacking technique in AR. Developed within the research of Cluster ENAC 2022 (EPFL). And thanks to the work of Andrea Settimi (IBOIS, EPFL), Ando Edward Carlo Giorgio (Imaging Analysis Center, EPFL), Qianqing Wang (EESD, EPFL) and the coordination of Julien Gamerro (IBOIS, EPFL)..


---

## 2022.02.03 - Installation of set-up + point cloud grabber

![screen_pcd_grabber](./img/grabber.png)

Installation of preliminary set-up one (rgbd + projector + metal frame + cables)

Also tested the pcd grabber and viewer with ZED sdk.


---

## 2022.02.04 - First draft of calibration between rgbd camera and projector

![calib1](./img/IMG_20220204_101414.jpg)

![calib2](./img/IMG_20220204_122843.jpg)

First functioning draft of the calibration process between the projector and the rgbd camera is done. 

To do list for the next steps:
* Try to find better cable?
* Nicer calibration object
* Measure a number of positions on a given (and recorded!! grid)
* Find out what this mysterious 4th field is from the output
* Get head around x=PX