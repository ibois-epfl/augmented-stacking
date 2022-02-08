#!/usr/bin/env python

import sys
import pyzed.sl as sl
import numpy as np


def main():

    print("Running Augmented Stacking depth capture GL viewer... Press 'Esc' to quit")

    # Set ZED params
    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, # HD720 | 1280*720
                             camera_fps=30, # available framerates: 15, 30, 60 fps
                             depth_mode=sl.DEPTH_MODE.QUALITY, # posible mods: sl.DEPTH_MODE.PERFORMANCE/.QUALITY/.ULTRA
                             coordinate_units=sl.UNIT.METER,
                             coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP, # sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP
                             sdk_verbose = True) # Enable verbose logging
    
    # Open ZED
    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    

    zed.close()


if __name__ == "__main__":
    main()

