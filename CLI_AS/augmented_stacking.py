#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Thu Feb 26 15:20:00 2022
CLI main file for Augmented Stacking:
The main file is used to run the augmented stacking algorithm to guide an operator in 
correctly placing stones following a stacking algorithm.
"""

import os
import sys
import subprocess

import dataset_IO
import stacking_algorithm
import distance_map
from util import terminal
from util import visualizer
import camera_capture

import numpy as np
import open3d as o3d
import colorama as cr

_name_landscape = './landscape.ply'

# Set a target of mesh faces for the low-res mesh
_faces_target_low_res_mesh = 500

# Set a target of max vertices for the captured scene mesh
_vertices_target_low_res_scene = 2000





def main():

    # [0] Decorator for CLI
    # V - [1] Download the low-res mesh
    # V - [2] Compute the mesh 6dof pose and update landscape
    # V - [3] Store/save the 6dof pose locally
    # V - [4] Load the high-res mesh
    # >>> [5] Capture scan with correct orientation to point cloud
    # [6] Calculate deviation
    # [7] Colored captured point cloud with rgb deviation values
    # [8] Pass it to the 3D-2D pipeline

    #-----------------------------------------------------------------------
    # [0] Decorator
    #-----------------------------------------------------------------------

    terminal.display_logo()
    print('\n<<<<<<<< INFORMATION PROJECT>>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')

    while(True):

        #-----------------------------------------------------------------------
        # [1] Download the low-res mesh
        #-----------------------------------------------------------------------

        # Ask for stone label & download LOW-RES mesh and open it
        name_low_res_mesh, low_res_mesh = dataset_IO.download_github_raw_file(is_raw_mesh=True)

        # Check the faces for the download mesh if not downsample
        faces_low_res_mesh = len(np.asarray(low_res_mesh.triangles))
        if (faces_low_res_mesh > _faces_target_low_res_mesh):
            terminal.custom_print(f"The mesh needs to be decimate for the stacking algorithms.\n"
                                  f" Number of vertices for downloaded mesh: {faces_low_res_mesh}")
            low_res_mesh = low_res_mesh.simplify_quadric_decimation(target_number_of_triangles=_faces_target_low_res_mesh)
            new_faces_low_res_mesh = len(np.asarray(low_res_mesh.triangles))
            terminal.custom_print(f" Number of vertices of the decimated mesh: {new_faces_low_res_mesh}")
            visualizer.viualize_mesh_normal(low_res_mesh, 'Low-res mesh')

        # Write out the mesh (for algorithm to read)
        o3d.io.write_triangle_mesh(name_low_res_mesh, low_res_mesh)


        #-----------------------------------------------------------------------
        # [2-3] Capture the scene mesh + compute the mesh 6dof pose
        #-----------------------------------------------------------------------

        # Capture meshed scene
        landscape_mesh = camera_capture.get_mesh_scene(_vertices_target_low_res_scene)
        visualizer.viualize_wall([landscape_mesh], 'wall view')

        # Save meshed scene
        print("Writing out the captured mesh from 3d camera")
        o3d.io.write_triangle_mesh(_name_landscape, landscape_mesh, write_ascii=True)
        print(f"Mesh out with name {_name_landscape}")

        # TODO: C++ reads from in-memory mesh
        # Compute stacking pose
        stacking_algorithm.compute(path_exec='./stacking_algorithm/build/main',
                                path_mesh=name_low_res_mesh,
                                path_landscape=_name_landscape, # TODO: param this
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
        visualizer.viualize_wall([low_res_mesh, landscape_mesh], 'wall view')



        #---------------------------------------------------------------------------
        #---------------------------------------------------------------------------
        #-----------------------------------------------------------------------
        # [4] Load the high-res mesh + apply 4x4 transformation
        #-----------------------------------------------------------------------

        # Download the HIGH-RES mesh file and open it
        name_highres_mesh, high_res_mesh = dataset_IO.download_github_raw_file(is_raw_mesh=False)

        # Apply transformation to mesh
        high_res_mesh.transform(pose_matrix)



        
        # First open the camera and close at the end
        zed, point_cloud = camera_capture.set_up_zed()

        # TEST: prepare visualizer
        pcd = camera_capture.get_pcd_scene(2000, zed, point_cloud)
        deviation_pc = distance_map.compute(mesh=high_res_mesh, pc=pcd)
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(deviation_pc)
        vis.add_geometry(high_res_mesh)

        # MIAN ADJUSTING LOOP
        # Now adjust the position untill it's in the good spot
        while(True):
            
            #-----------------------------------------------------------------------
            # [5] Calculate deviation from captured point cloud
            #-----------------------------------------------------------------------

            # Get point cloud from camera
            pcd = camera_capture.get_pcd_scene(2000, zed, point_cloud)

            # Calculate the deviation of the rock from the cloud
            pcd_temp = distance_map.compute(mesh=high_res_mesh, pc=pcd)
            deviation_pc.points = pcd_temp.points
            deviation_pc.colors = pcd_temp.colors

            vis.update_geometry(deviation_pc)
            vis.poll_events()
            vis.update_renderer()

            # TODO: implement non-block visualization open3d
            # high_res_mesh.compute_vertex_normals() # // DEBUG
            # visualizer.viualize_wall([deviation_pc, high_res_mesh], 'wall view')
        
        # Destroy non-blocking visualizator
        vis.destroy_window()

        # Now that the loop is closed, close the camera
        zed.close()

    #-----------------------------------------------------------------------
    # TODO LIST
    #-----------------------------------------------------------------------

    #TODO: (Andrea) write code to capture pointcloud with origin in the center
    #TODD: (Qianqing) adjust the sensitivity of the deviance map coloring
    #TODO: (Qianqing) for deviance map conider only local area (e.g. sphere around the pointcloud)
    #TODO: (Andrea) wrap everything in a wrap loop



if __name__ == "__main__":
    main()