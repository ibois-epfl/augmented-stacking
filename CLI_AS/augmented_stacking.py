#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Thu Feb 26 15:20:00 2022
CLI main file for Augmented Stacking
"""

import os
import sys
import subprocess

import dataset_IO
import stacking_algorithm

import numpy as np
import open3d as o3d


_url_low_res_dir = 'https://raw.githubusercontent.com/ibois-epfl/augmented-stacking-dataset/main/stones/low_res/'
_url_high_res_dir = 'https://raw.githubusercontent.com/ibois-epfl/augmented-stacking-dataset/main/stones/high_res/'
# _name_mesh = '073ac15d-8ad2-4758-9c8c-5f8339b97edb_Mesh.ply'
_name_lowres_mesh = 'stone_0.ply'
_name_highres_mesh = 'stone_0_high_res.ply'

_name_landscape = 'landscape.ply'


def main():

    # V - [1] Download the low-res mesh
    # V - [2] Compute the mesh 6dof pose and update landscape
    # V - [3] Store/save the 6dof pose locally
    # >>> [4] Load the high-res mesh
    # [5] Calculate deviation
    # [6] Colored captured point cloud with rgb deviation values
    # [8] Pass it to the 3D-2D pipeline

    #-----------------------------------------------------------------------
    # [1] Download the low-res mesh + apply 4x4 transformation
    #-----------------------------------------------------------------------

    # Download the LOW-RES mesh file
    dataset_IO.download_github_raw_file(_url_low_res_dir, _name_lowres_mesh)
    
    # # Open and display mesh with open3d
    # mesh = o3d.io.read_triangle_mesh(_name_lowres_mesh)
    # o3d.visualization.draw_geometries([mesh])


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
                               path_mesh=_name_lowres_mesh,
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

    




if __name__ == "__main__":
    main()