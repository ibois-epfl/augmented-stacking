#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Thu Feb 26 15:20:00 2022
CLI main file for Augmented Stacking:
The main file is used to run the augmented stacking algorithm to guide an operator in
correctly placing stones following a stacking algorithm and an AR system composed by a 
projector and 3D camera (ZEDi2)
"""


import os
import sys
import subprocess

# TODO: the package opencv is not found in virtual env, solve
# the issue without importing the sys.path here. With this it works
sys.path.append('/usr/local/lib/python3.8/dist-packages')
import cv2

import dataset_IO
import stacking_algorithm
import distance_map
from util import terminal
from util import visualizer
import camera_capture
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import colorama as cr



# TODO: implement non-block visualization open3d

# Path where the landscape is overwritten
_name_landscape = './landscape.ply'

# Set a target of mesh faces for the low-res mesh
_faces_target_low_res_mesh = 500

# Set a target of max vertices for the captured scene mesh
_vertices_target_low_res_scene = 2000

#TODO: solve the conflict between o3d visualizer and tkinter, temp var
_is_o3d_vis_active = False


def main():

    # -----------------------------------------------------------------------
    # [0] Decorator
    # -----------------------------------------------------------------------

    terminal.display_logo()
    print('\n<<<<<<<< INFORMATION PROJECT>>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')

    while(True):

        # -----------------------------------------------------------------------
        # [1] Download the low-res mesh
        # -----------------------------------------------------------------------

        while(True):
            # Ask for stone label & download LOW-RES mesh and open it
            name_low_res_mesh, low_res_mesh = dataset_IO.download_github_raw_file()
            
            #TODO: solve the conflict between o3d visualizer and tkinter, temp
            i_o3d_vis_active = terminal.user_input('Do you want to visualize Pcd in O3D ? (y/n)\n>>> ')
            if i_o3d_vis_active in ["y","yn"]: _is_o3d_vis_active = True

            # Check the n faces for the download mesh if not downsample
            faces_low_res_mesh = len(np.asarray(low_res_mesh.triangles))
            if (faces_low_res_mesh > _faces_target_low_res_mesh):
                terminal.custom_print(f"The mesh needs to be decimate for the stacking algorithms.\n"
                                    f" Number of vertices for downloaded mesh: {faces_low_res_mesh}")
                low_res_mesh = low_res_mesh.simplify_quadric_decimation(
                    target_number_of_triangles=_faces_target_low_res_mesh)
                new_faces_low_res_mesh = len(np.asarray(low_res_mesh.triangles))
                terminal.custom_print(f" Number of vertices of the decimated mesh: {new_faces_low_res_mesh}")
            
            # Visual inspection of downloaded mesh
            if _is_o3d_vis_active:
                visualizer.viualize_mesh_normal(low_res_mesh, 'Low-res mesh') # TODO: Fix the issue of using both open3d and tkinter
            
            # Ask if you want to select stone or chose another one
            i_is_stone_correct = terminal.user_input('Do you confirm this stone (if n you will enter the label again)? (y/n)\n>>> ')
            if i_is_stone_correct in ['Y', 'y']: break

        # Write out the mesh (for algorithm to read)
        o3d.io.write_triangle_mesh(name_low_res_mesh, low_res_mesh)

        exit()

        # -----------------------------------------------------------------------
        # [2-3] Capture the scene mesh + compute the mesh 6dof pose
        # -----------------------------------------------------------------------

        # Capture meshed scene
        landscape_mesh = camera_capture.get_mesh_scene(
            _vertices_target_low_res_scene)
        
        if _is_o3d_vis_active:
            visualizer.viualize_wall([landscape_mesh], 'wall view')

        # # Save meshed scene
        print("Writing out the captured mesh from 3d camera")
        o3d.io.write_triangle_mesh(
            _name_landscape, landscape_mesh, write_ascii=True)
        print(f"Mesh out with name {_name_landscape}")

        # TODO: C++ reads from in-memory mesh
        # Compute stacking pose
        stacking_algorithm.compute(path_exec='./stacking_algorithm/build/main',
                                   path_mesh=name_low_res_mesh,
                                   path_landscape=_name_landscape,  # TODO: param this
                                   config_file='./stacking_algorithm/data/input.txt',
                                   name_output='pose',
                                   dir_output='./temp')

        # TEMP: Delete donwload mesh
        dataset_IO.delete_file(name_low_res_mesh)

        # read the 6dof pose and store it as numpy array
        pose_matrix = dataset_IO.read_pose_6dof('./temp/pose.txt')
        print("Computation done: here's the computed 4x4 Pose Matrix:\n\n", pose_matrix)

        # Transform the low-res mesh for visualization
        low_res_mesh.transform(pose_matrix)
        
        if _is_o3d_vis_active:
            visualizer.viualize_wall([low_res_mesh, landscape_mesh], 'wall view')

        # -----------------------------------------------------------------------
        # [4] Load the high-res mesh + apply 4x4 transformation
        # -----------------------------------------------------------------------

        # Merge transformed stone and landscape mesh
        merged_landscape = low_res_mesh + landscape_mesh

        if _is_o3d_vis_active:
            visualizer.viualize_wall([merged_landscape],"merged landscape")
        # First open the camera and close at the end
        zed, point_cloud = camera_capture.set_up_zed()

        # Create the 3D space model
        Live_3D_space = camera_capture.Live_3D_space(rock_mesh=low_res_mesh,zed=zed,point_cloud=point_cloud)

        # create image object 
        image_drawer = camera_capture.Image_drawer(Live_3D_space=Live_3D_space)
        
        if _is_o3d_vis_active:
            terminal.custom_print(f"When the stone is placed correnctly, Press <Escape> to close the tkinter window.")
            Live = camera_capture.Live_stream(Live_3D_space=Live_3D_space,image_drawer=image_drawer)
            Live.run()
        
        # Now that the loop is closed, close the camera
        zed.close()  #TODO: check if we need to close the zed each time or we can continue

    # -----------------------------------------------------------------------
    # TODO LIST
    # -----------------------------------------------------------------------

    # TODO: (Andrea) write code to capture pointcloud with origin in the center
    # TODD: (Qianqing) adjust the sensitivity of the deviance map coloring
    # TODO: (Qianqing) for deviance map conider only local area (e.g. sphere around the pointcloud)
    # TODO: (Andrea) wrap everything in a wrap loop


if __name__ == "__main__":
    main()
