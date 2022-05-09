
'''
The module implements I/O functionalities for augmented stacking
'''

import requests
import os
import sys
import tqdm
import numpy as np
from util import terminal
import datetime

import open3d as o3d

global STONE_COUNTER


def download_github_raw_file():
    """
    Download a file from GitHub, don't forget to put the raw http of github file
    e.g. https://raw.githubusercontent.com/ibois-epfl/augmented-stacking-dataset/main/stones/low_res/bun_zipper.ply
    The function loops until a good label for an existing rock is given as input.
    The rock is than store in memory as o3d mesh format.

    return: string of the namefile, mesh
    """
    # GitHub Raw version of dataset address
    URL_MESH_DIR = 'https://raw.githubusercontent.com/ibois-epfl/augmented-stacking-dataset/main/stones/mesh_high_res/'

    # Global variable to store the label of the stone
    global stone_label 

    # Loop until a good label is given
    while True:
        # Ask for user input
        stone_label = terminal.user_input('Enter the stone label: \n>>> ')
        filename = f'remeshed_high_res_{stone_label}.ply'
        url_path = os.path.join(URL_MESH_DIR, filename)
        print(f'>>> Downloading the mesh: {filename}')

        # Check if the file is already downloaded
        if os.path.isfile(filename):
            print(f"File {filename} already downloaded")
            mesh = o3d.io.read_triangle_mesh(filename)
            delete_file(filename)
            return filename, mesh

        # Download the file and write .ply file
        try:
            with tqdm.tqdm(unit='A', unit_scale=True, miniters=1, desc=filename) as t:
                r = requests.get(url_path, stream=True)
                if r.status_code == 404:
                    terminal.error_print(f"File {filename} not found in the dataset")
                    continue
                else:
                    total_size = int(r.headers.get('content-length', 0))
                    block_size = 1024
                    wrote = 0
                    with open(filename, 'wb') as f:
                        for data in r.iter_content(block_size):
                            wrote = wrote + len(data)
                            f.write(data)
                            t.update(len(data))
                    print(f'\n>>> File {filename} successfully downloaded and imported')
                    mesh = o3d.io.read_triangle_mesh(filename)
                    delete_file(filename)
                    return filename, mesh
        except Exception as e:
            terminal.error_print(f"Error while downloading {filename}")
            terminal.error_print(e)
            sys.exit(1)

def delete_file(filename):
    """
    Erase a particular file

    :param url: filename path
    """
    if os.path.isfile(filename):
        os.remove(filename)

def read_pose_6dof(filename):
    """
    Read a 6dof pose from a text file containing a matrix and return a numpy matrix
    Here's what the .txt file looks like:

     -0.148643 0.00148648   -0.98889   0.168563
     0.465496   0.882384 -0.0686438   0.136784
     0.872479  -0.470527  -0.131852  0.0631906
             0          0          0          1

    :param filename: path of the text file

    :return: numpy 4x4 matrix with the 6dof pose
    """

    # Check if the file exists
    if not os.path.isfile(filename):
        print(f"Pose text file {filename} does not exist")
        return
    
    # Read the text file
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    # Store values in numpy transform matrix
    pose_6dof = np.zeros((4, 4))
    for i in range(4):
        pose_6dof[i, :] = [float(x) for x in lines[i].split()]

    return pose_6dof

def save_current_built_layer(landscape_mesh,
                             stone_mesh,
                             stone_label:str,
                             sub_dir:os.path,
                             stone_counter:int) -> None:
    """
    Save all the geometries of the as-built wall status for later inspection 
    or live visualization.

    :param landscape_mesh: scene of the building landscape prior to stacking algorithm
    :param stone_mesh: stone mesh placed by stacking algorithm
    :param stone_label: label of the stone from dataset
    :param sub_dir: recording session subdir

    :return: the updated stone counter of the current stone
    """
    # Update stone counter for recording tracking
    stone_counter += 1

    # Write out placed stone mesh on stacking algorithm
    stone_label = stone_label.split('_')[-1].split('.')[0]
    path_stone_mesh = os.path.join(sub_dir, f"{stone_counter}_stone_mesh_{stone_label}.ply")
    o3d.io.write_triangle_mesh(path_stone_mesh, stone_mesh)
    terminal.custom_print(f"Dropped placed stone {stone_label} on folder {sub_dir}")

    # Scene of the building landscape prior to stacking algorithm
    path_landscape_mesh = os.path.join(sub_dir, f"{stone_counter}_scene_mesh_{stone_label}.ply")
    o3d.io.write_triangle_mesh(path_landscape_mesh, landscape_mesh)
    terminal.custom_print(f"Dropped corresponding scene capture on folder {sub_dir}")

    return stone_counter, path_stone_mesh, path_landscape_mesh

def create_record_session_subdir(dir_name:os.path) -> os.path:
    if os.path.exists(dir_name):
        current_date = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        subdir = os.path.join(dir_name, current_date)
        os.mkdir(subdir)
        return subdir
    else:
        terminal.error_print(f"The dir {dir_name} does not exist.")
        return None