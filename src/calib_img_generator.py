""" 
usage: calib_img_generator.py [-h] [-x NBLINESX] [-y NBLINESY] [-w LINEWIDTH]
Create a calibration image.
optional arguments:
  -h, --help                                    Show this help message and exit.
  -x NBLINESX, --nbLinesX NBLINESX              Number of Lines on X axis.
  -y NBLINESY, --nbLinesY NBLINESY              Number of Lines on Y axis.
  -w LINEWIDTH, --lineWidth LINEWIDTH           Width of the Line drawn in pixels.
"""


import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont
import os
import argparse
from datetime import date
from pathlib import Path

def draw_grid(nb_lines_X=3,nb_lines_Y=3,line_width=4):
    X =[]
    Y =[]
    # Initialize black image
    shape=(1080,1920,3)
    Img = np.zeros(shape,dtype=np.uint8)

    # Calculate space between lines
    X_space = (shape[0] - nb_lines_X*line_width)//(nb_lines_X+1)
    Y_space = (shape[1] - nb_lines_Y*line_width)//(nb_lines_Y+1)

    #Pts coordinate saving
    Pts=np.zeros((nb_lines_Y*nb_lines_X,2))

    # Draw the lines
    for i in range(1,nb_lines_X+1):
        Img[i*X_space:i*X_space+line_width+1,:,1]=255
        for j in range (1,nb_lines_Y+1):
            Pts[(i-1)*(nb_lines_X+1)+(j-1),1]=j*Y_space+line_width//2
            Pts[(i-1)*(nb_lines_X+1)+(j-1),0]=i*X_space+line_width//2
    for j in range(1,nb_lines_Y+1):
        Img[:,i*Y_space:i*Y_space+line_width+1,1]=255

    
    return Img,Pts

def main(nb_lines_X,nb_lines_Y,line_width):
    grid,Pts = draw_grid(nb_lines_X=nb_lines_X,nb_lines_Y=nb_lines_Y,line_width=line_width)
    np.save("Pixel_pts",Pts)
    imgPath="calibration_grid.png"
    plt.imsave(imgPath,grid)

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description="Create a Calibration Image",
                                   formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        '-x', '--nbLinesX',
        help="Number of Line on X axis.",
        type=int,
        default=3
    )
    parser.add_argument(
        '-y', '--nbLinesY',
        help="Number of Line on Y axis.",
        type=int,
        default=3
    )
    parser.add_argument(
        '-w', '--lineWidth',
        help="Width of the line drawn in pixels.",
        type=int,
        default=4
    )
    args = parser.parse_args()

    
    main(args.nbLinesY, args.nbLinesX, args.lineWidth)
