## This readme file is explain the live image script, which can be used to display live 3D data on the scene using the calibrated projector.

For this script you will need following files:

-  3D_2D_matrix.npy

This file location is hard coded in the script, just change the location in line 16.


And you will also need to define a live function you want to use.
For now the live function used is acquiring point clouds from the zed camera and displaying them with the projector.
Every object which will appear in the scene, will be projected with a green image of it self.



Just launch the script with following command 

```bash
python Live_img.py
```
