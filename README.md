# augmented-stacking


<p align="center">
    <img src="./logo/whitelogo.png" width="300">
</p>

<br />

Development code for an `dry-stone` and `-timber` stacking technique in AR. `Augmented Stacking` is a `CLI` software developed and tested on `Ubuntu 20.04 LTS`.

Augmented Stacking is an augmented system designed to guide unexperienced users through the stacking of complex and irregular geometries. This digital fabrication system was applied to the assembly of one layer of half-timber dry-stone wall. Augmented Stacking is composed of two components: the stacking algorithm (which calculates the pose for each stone or irregular object feeded to the system) and the augmented pipeline to display the information. 

<p>
    <img src="./img/demo_placing_example_light.gif" width="600">
</p>

<p>
    <img src="./img/augmented_stones_finalShot_v1.png" width="600">
</p>


# To run the code

**[0]** The software is made to be used with any type of LED projector. Nevertheless you will need a camera [ZED2i](https://www.stereolabs.com/zed-2i/) to work with the current version. The code can be easily adapted to any type of 3D sensor. The address of your digitized library of digital-twins to assemble need to be replaced in ``

**[1]** Install the ZED2i [SDK](https://download.stereolabs.com/zedsdk/3.7/cu115/ubuntu20)

**[2]** Run the virtual environment with `fish`:
```bash
source ./venv/bin/activate
```

**[3]** Run a calibration and follow the instraction in the console:
```bash
python ./calib/calib.py
```

**[4]** Run augmented stacking:
```bash
python ./calib/calib.py
```
