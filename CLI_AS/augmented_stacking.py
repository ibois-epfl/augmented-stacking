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

import numpy as np
import open3d as o3d
import colorama as cr


_url_low_res_dir = 'https://raw.githubusercontent.com/ibois-epfl/augmented-stacking-dataset/main/stones/low_res/'
_url_high_res_dir = 'https://raw.githubusercontent.com/ibois-epfl/augmented-stacking-dataset/main/stones/high_res/'
# _name_mesh = '073ac15d-8ad2-4758-9c8c-5f8339b97edb_Mesh.ply'
_name_highres_mesh = 'high_res_0.ply'

_name_landscape = 'landscape.ply'




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
    print('\n<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')



    #-----------------------------------------------------------------------
    # [1] Download the low-res mesh
    #-----------------------------------------------------------------------

    # Ask for stone label & download LOW-RES mesh
    name_low_res_mesh = dataset_IO.download_github_raw_file(_url_low_res_dir)
    low_res_mesh = o3d.io.read_triangle_mesh(name_low_res_mesh)
    print(f'>>> Mesh {name_low_res_mesh} successfully downloaded and imported')


    # Display mesh
    terminal.custom_print('>>> Press [Esc] to continue ...')
    visualizer.viualize_mesh_normal(low_res_mesh, 'Low-res mesh')


    #-----------------------------------------------------------------------
    # [2-3] Compute the mesh 6dof pose and update landscape
    #-----------------------------------------------------------------------

    # Harcoded dirs/paths (TODO: make them configurable)
    _dir_cpp_exec = './stacking_algorithm'
    _path_exec = f'{_dir_cpp_exec}/build/main'
    _config_file = f'{_dir_cpp_exec}/data/input.txt'
    _name_output = 'pose'
    _dir_output = './temp'

    # Compute stacking pose
    stacking_algorithm.compute(path_exec=_path_exec,
                               path_mesh=name_low_res_mesh,
                               path_landscape=_name_landscape,
                               config_file=_config_file,
                               name_output=_name_output,
                               dir_output=_dir_output)

    # read the 6dof pose and store it as numpy array
    pose_matrix = dataset_IO.read_pose_6dof('./temp/pose.txt')
    print("4x4 Pose Matrix:\n", pose_matrix)


    #-----------------------------------------------------------------------
    # [4] Load the high-res mesh + apply 4x4 transformation
    #-----------------------------------------------------------------------

    # Download the HIGH-RES mesh file
    dataset_IO.download_github_raw_file(_url_high_res_dir, _name_highres_mesh)

    # Apply transformation to mesh
    high_res_mesh = o3d.io.read_triangle_mesh(_name_highres_mesh)
    high_res_mesh.transform(pose_matrix)

    # Write mesh open3d ascii file
    o3d.io.write_triangle_mesh('testhighrestransformed.ply', high_res_mesh, write_ascii=True)


    #-----------------------------------------------------------------------
    # [5] Calculate deviation from captured point cloud
    #-----------------------------------------------------------------------

    _path_pc_captured_landscape = 'A46_point_cloud.ply'
    _pc_captured_landscape = o3d.io.read_point_cloud(_path_pc_captured_landscape)



    # Get the center of the pointcloud
    center = _pc_captured_landscape.get_center()
    # Move poincloud to origin
    _pc_captured_landscape.translate(-center)
    # Rotate pointcloud of 90 degrees around z axis
    R = _pc_captured_landscape.get_rotation_matrix_from_xyz([0, 0, np.pi/2])
    _pc_captured_landscape.rotate(R, center=(0,0,0))



    deviation_pc = distance_map.compute(mesh=high_res_mesh, pc=_pc_captured_landscape)



    landscape = o3d.io.read_triangle_mesh(_name_landscape)

    # Create sphere on origin
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)




    o3d.visualization.draw_geometries([deviation_pc, high_res_mesh, landscape, sphere])


    #-----------------------------------------------------------------------
    # TODO LIST
    #-----------------------------------------------------------------------

    #TODO: (Andrea) write code to capture pointcloud with origin in the center
    #TODD: (Qianqing) adjust the sensitivity of the deviance map coloring
    #TODO: (Qianqing) for deviance map conider only local area (e.g. sphere around the pointcloud)
    #TODO: (Andrea) wrap everything in a wrap loop



if __name__ == "__main__":
    main()