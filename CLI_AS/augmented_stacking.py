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
# _name_mesh = '073ac15d-8ad2-4758-9c8c-5f8339b97edb_Mesh.ply'
_name_mesh = 'stone_0.ply'
_name_landscape = 'landscape.ply'


def main():

    # V - [1] Download the low-res mesh
    # >>> [2] Compute the mesh 6dof pose and update landscape
    # [3] Store/save the 6dof pose locally
    # [4] Load the high-res mesh
    # [5] Calculate deviation
    # [6] Colored captured point cloud with rgb deviation values
    # [8] Pass it to the 3D-2D pipeline

    #-----------------------------------------------------------------------
    # [1] Download the low-res mesh
    #-----------------------------------------------------------------------

    # Download the mesh file
    dataset_IO.download_github_raw_file(_url_low_res_dir, _name_mesh)
    
    # Open and display mesh with open3d
    mesh = o3d.io.read_triangle_mesh(_name_mesh)
    o3d.visualization.draw_geometries([mesh])


    #-----------------------------------------------------------------------
    # [2] Compute the mesh 6dof pose and update landscape
    #-----------------------------------------------------------------------

    # Harcoded dirs/paths (TODO: make them configurable)
    _dir_cpp_exec = './stacking_algorithm'
    _path_exec = f'{_dir_cpp_exec}/build/main'
    _config_file = f'{_dir_cpp_exec}/data/input.txt'
    _name_output = 'pose'
    _dir_output = './temp'

    # Compute stacking pose
    stacking_algorithm.compute(path_exec=_path_exec,
                               path_mesh=_name_mesh,
                               path_landscape=_name_landscape,
                               config_file=_config_file,
                               name_output=_name_output,
                               dir_output=_dir_output)

    # read the 6dof pose and store it as numpy array
    test_matrix = dataset_IO.read_pose_6dof('./temp/pose.txt')
    print(test_matrix)

if __name__ == "__main__":
    main()