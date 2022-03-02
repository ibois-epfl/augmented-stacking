
'''
The module implements I/O functionalities for augmented stacking
'''

import requests
import os
import sys
import tqdm
import numpy as np


def download_github_raw_file(url, filename):
    """
    Download a file from GitHub, don't forget to put the raw http of github file
    e.g. https://raw.githubusercontent.com/ibois-epfl/augmented-stacking-dataset/main/stones/low_res/bun_zipper.ply

    :param url: url of the file
    :param filename: name of the file
    :return:

    """

    # Check if the file is already downloaded
    if os.path.isfile(filename):
        print(f"File {filename} already downloaded")
        return

    # Download the file and write .ply file
    try:
        url_path = os.path.join(url, filename)
        with tqdm.tqdm(unit='A', unit_scale=True, miniters=1, desc=filename) as t:
            r = requests.get(url_path, stream=True)
            # Total size in bytes.
            total_size = int(r.headers.get('content-length', 0))
            block_size = 1024
            wrote = 0
            with open(filename, 'wb') as f:
                for data in r.iter_content(block_size):
                    wrote = wrote + len(data)
                    f.write(data)
                    t.update(len(data))
    except Exception as e:
        print(f"Error while downloading {filename}")
        print(e)
        sys.exit(1)
    
    return

def delete_file(filename):
    """
    Erase a particular file

    :param url: filename path
    :return:

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
