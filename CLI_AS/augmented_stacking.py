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

_name_landscape = './landscape.ply'


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

    # -----------------------------------------------------------------------
    # [0] Decorator
    # -----------------------------------------------------------------------

    terminal.display_logo()
    print('\n<<<<<<<< INFORMATION PROJECT>>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')
    print('<<<<<<<< INFORMATION PROJECT >>>>>>>>')

<<<<<<< HEAD
    # -----------------------------------------------------------------------
    # [1] Download the low-res mesh
    # -----------------------------------------------------------------------

    # Ask for stone label & download LOW-RES mesh and open it
    name_low_res_mesh, low_res_mesh = dataset_IO.download_github_raw_file(
        is_raw_mesh=True)

    # Display mesh
    terminal.custom_print('>>> Press [Esc] to continue ...')
    visualizer.viualize_mesh_normal(low_res_mesh, 'Low-res mesh')
    o3d.io.write_triangle_mesh(name_low_res_mesh, low_res_mesh)

    # -----------------------------------------------------------------------
    # [2-3] Compute the mesh 6dof pose and update landscape
    # -----------------------------------------------------------------------

    # Compute stacking pose
    NBPOSE = 5
    global stack_try

    def stack_try(n):
        config_file = './stacking_algorithm/data/input_'+str(n)+".txt"
        name_output = 'pose_'+str(n)
        stacking_algorithm.compute(path_exec='./stacking_algorithm/build/main',
                                   path_mesh=name_low_res_mesh,
                                   path_landscape=_name_landscape,  # TODO: param this
                                   config_file=config_file,
                                   name_output=name_output,
                                   dir_output='./temp')

        # read the 6dof pose and store it as numpy array
        pose_matrix = dataset_IO.read_pose_6dof(f'./temp/pose_{n}.txt')
        print("Computation done: here's the computed 4x4 Pose Matrix:\n\n", pose_matrix)
        low_res_mesh_copy = o3d.geometry.TriangleMesh(low_res_mesh)
        low_res_mesh_copy.transform(pose_matrix)
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.1, origin=[0, 0, 0])
        landscape = o3d.io.read_triangle_mesh(_name_landscape)
        visualizer.viualize_wall(
            [low_res_mesh_copy, landscape, axis], f'{n} wall view')
        return pose_matrix
    from multiprocessing import Pool
    pool = Pool(5)
    transforms = pool.map(stack_try, list(range(NBPOSE)))
    choice = input('>>> Choose a pose ...')
    pose_matrix = transforms[int(choice)]
    # -----------------------------------------------------------------------
=======
    while(True):


        #-----------------------------------------------------------------------
        # [1] Download the low-res mesh
        #-----------------------------------------------------------------------

        # Ask for stone label & download LOW-RES mesh and open it
        name_low_res_mesh, low_res_mesh = dataset_IO.download_github_raw_file(is_raw_mesh=True)

        o3d.io.write_triangle_mesh(name_low_res_mesh, low_res_mesh)
        print(f'DEBUG {name_low_res_mesh}')

        # Display mesh
        terminal.custom_print('>>> Press [Esc] to continue ...')
        visualizer.viualize_mesh_normal(low_res_mesh, 'Low-res mesh')


        #-----------------------------------------------------------------------
        # [2-3] Compute the mesh 6dof pose and update landscape
        #-----------------------------------------------------------------------

        # Compute stacking pose
        stacking_algorithm.compute(path_exec='./stacking_algorithm/build/main',
                                path_mesh=name_low_res_mesh,
                                path_landscape=_name_landscape, # TODO: param this
                                config_file='./stacking_algorithm/data/input.txt',
                                name_output='pose',
                                dir_output='./temp')

        # read the 6dof pose and store it as numpy array
        pose_matrix = dataset_IO.read_pose_6dof('./temp/pose.txt')
        print("Computation done: here's the computed 4x4 Pose Matrix:\n\n", pose_matrix)

        # DEBUG import landscape
        temp_landscape = o3d.io.read_triangle_mesh(_name_landscape)
        temp_landscape.compute_vertex_normals()

        low_res_mesh.transform(pose_matrix)

        print(f'DEBUG::MIN POINT LANDSCAPE: {np.min(temp_landscape.vertices)}')


        # DEBUG: merge stone cloud with landscape and print out landscape
        # mergedmesh = low_res_mesh + temp_landscape
        mergedmesh = low_res_mesh + temp_landscape
        visualizer.viualize_wall([mergedmesh], 'wall view')

        # DEBUG: replace landscape mesh with the new one
        o3d.io.write_triangle_mesh(_name_landscape, mergedmesh)


        




    print("BREAKPOINT")
    exit()

    #-----------------------------------------------------------------------
>>>>>>> 05187f97be414cf9cebea38ef2f024024a69c81c
    # [4] Load the high-res mesh + apply 4x4 transformation
    # -----------------------------------------------------------------------

    # Download the HIGH-RES mesh file and open it
    name_highres_mesh, high_res_mesh = dataset_IO.download_github_raw_file(
        is_raw_mesh=False)

    # Apply transformation to mesh
    high_res_mesh.transform(pose_matrix)

    # -----------------------------------------------------------------------
    # [5] Calculate deviation from captured point cloud
    # -----------------------------------------------------------------------

    _path_pc_captured_landscape = 'A46_point_cloud.ply'
    _pc_captured_landscape = o3d.io.read_point_cloud(
        _path_pc_captured_landscape)

    # # Get the center of the pointcloud
    # center = _pc_captured_landscape.get_center()
    # # Move poincloud to origin
    # _pc_captured_landscape.translate(-center)

    # # Rotate pointcloud of 90 degrees around z axis // DEBUG
    # R = _pc_captured_landscape.get_rotation_matrix_from_xyz([0, 0, np.pi/2])
    # _pc_captured_landscape.rotate(R, center=(0,0,0))

    deviation_pc = distance_map.compute(
        mesh=high_res_mesh, pc=_pc_captured_landscape)

    landscape = o3d.io.read_triangle_mesh(_name_landscape)

    # Draw axis blue, green and red
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.1, origin=[0, 0, 0])
    # o3d.visualization.draw_geometries([deviation_pc, high_res_mesh, landscape, axis])
    landscape.compute_vertex_normals()  # // DEBUG
    high_res_mesh.compute_vertex_normals()  # // DEBUG

    visualizer.viualize_wall(
        [deviation_pc, high_res_mesh, landscape, axis], 'wall view')

    # -----------------------------------------------------------------------
    # TODO LIST
    # -----------------------------------------------------------------------

    # TODO: (Andrea) write code to capture pointcloud with origin in the center
    # TODD: (Qianqing) adjust the sensitivity of the deviance map coloring
    # TODO: (Qianqing) for deviance map conider only local area (e.g. sphere around the pointcloud)
    # TODO: (Andrea) wrap everything in a wrap loop


if __name__ == "__main__":
    main()
