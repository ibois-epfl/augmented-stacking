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
_faces_target_stone_mesh = 500

# Set a target of max vertices for the captured scene mesh
_vertices_target_low_res_scene = 2000



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

    print('Before to start make sure that:')
    print('You have done a calibration')
    print('You have a second screen with the same resolution as your main machine')




    while(True):

        # -----------------------------------------------------------------------
        # [1] Download the low-res mesh
        # -----------------------------------------------------------------------

        while(True):
            # Ask for stone label & download mesh and open it
            name_stone_mesh, stone_mesh = dataset_IO.download_github_raw_file()
            
            # Check the n faces for the download mesh if not downsample
            faces_stone_mesh = len(np.asarray(stone_mesh.triangles))
            if (faces_stone_mesh > _faces_target_stone_mesh):
                terminal.custom_print(f"The mesh needs to be decimate for the stacking algorithms.\n"
                                      f" Number of vertices for downloaded mesh: {faces_stone_mesh}")
                stone_mesh = stone_mesh.simplify_quadric_decimation(
                    target_number_of_triangles=_faces_target_stone_mesh)
                new_faces_stone_mesh = len(np.asarray(stone_mesh.triangles))
                terminal.custom_print(f" Number of vertices of the decimated mesh: {new_faces_stone_mesh}")
            
            # Check if the mesh is out of scale or not watertight, and confirm
            try:
                if stone_mesh.get_volume() > 1.0:  # 1m3
                    stone_mesh = stone_mesh.scale(scale=1/1000, 
                                                center=stone_mesh.get_center())
                
                i_is_stone_correct = terminal.user_input('Do you confirm this stone (if n you will enter the label again)? (y/n)\n>>> ')
                    if i_is_stone_correct in ['Y', 'y']: break

            except:
                terminal.error_print("ERROR: the imported mesh is corrupted or not water tight.\n",
                                     "Choose another stone from dataset ...")
                continue


        # Write out the mesh (for algorithm to read)
        o3d.io.write_triangle_mesh(name_stone_mesh, stone_mesh)


        # -----------------------------------------------------------------------
        # [2] Capture the scene mesh + compute the mesh 6dof pose
        # -----------------------------------------------------------------------

        # Capture meshed scene
        landscape_mesh = camera_capture.get_mesh_scene(
            _vertices_target_low_res_scene)
        
        if i_o3d_vis_active in ["y","yn"]:
            visualizer.viualize_wall([landscape_mesh], 'wall view')

        # # Save meshed scene
        print("Writing out the captured mesh from 3d camera")
        o3d.io.write_triangle_mesh(
            _name_landscape, landscape_mesh, write_ascii=True)
        print(f"Mesh out with name {_name_landscape}")

        # Compute stacking pose
        stacking_algorithm.compute(path_exec='./stacking_algorithm/build/main',
                                   path_mesh=name_stone_mesh,
                                   path_landscape=_name_landscape,  # TODO: param this
                                   config_file='./stacking_algorithm/data/input.txt',
                                   name_output='pose',
                                   dir_output='./temp')

        # TEMP: Delete donwload mesh
        dataset_IO.delete_file(name_stone_mesh)

        # read the 6dof pose and store it as numpy array
        pose_matrix = dataset_IO.read_pose_6dof('./temp/pose.txt')
        print("Computation done: here's the computed 4x4 Pose Matrix:\n\n", pose_matrix)

        # Transform the low-res mesh for visualization
        stone_mesh.transform(pose_matrix)
        
        
        # -----------------------------------------------------------------------
        # [3] Augmented feedback loop
        # -----------------------------------------------------------------------

        # Merge transformed stone and landscape mesh
        merged_landscape = stone_mesh + landscape_mesh

        if i_o3d_vis_active in ["y","yn"]:
            visualizer.viualize_wall([merged_landscape],"merged landscape")
        # First open the camera and close at the end
        zed, point_cloud = camera_capture.set_up_zed()

        # Create the 3D space model
        Live_3D_space = camera_capture.Live_3D_space(rock_mesh=stone_mesh,zed=zed,point_cloud=point_cloud)

        # create image object 
        image_drawer = camera_capture.Image_drawer(Live_3D_space=Live_3D_space)
        
        # Start live 2D projection to guide user to stone placement
        terminal.custom_print(f"When the stone is placed correnctly, Press <Escape> to close the tkinter window.")
        live_stream = camera_capture.Live_stream(Live_3D_space=Live_3D_space,image_drawer=image_drawer)
        live_stream.run()
        
        # Now that the loop is closed, close the camera
        zed.close()

        # Ask to validate the stone pose and terminate cli
        i_is_pose_stone_valid = terminal.user_input('Do you validate the stone position? (y/n)\n>>> ')
        if i_is_pose_stone_valid in ['Y', 'y']:
            #TODO: add the stone pose/matrix to a list of validated poses
            i_continue_stacking = terminal.user_input('Do you want to stack another stone? y/n)\n>>> ')
            if i_continue_stacking in ['N', 'n']:
                terminal.custom_print("The augmented stacking is shuting down ...")
                exit()


if __name__ == "__main__":
    main()
