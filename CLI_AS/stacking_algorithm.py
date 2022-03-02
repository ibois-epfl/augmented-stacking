
'''
Moule to call EESD stacking algorithm to compute the 6dof pose of the rock and update the landscape
For now the outputs are written to disk in ./temp folder because we don´t have pybinding
'''

import sys

def compute(path_exec, path_mesh, path_landscape, config_file, dir_output):
    """
    Call the stacking algorithm c++ file

    :param path_exec: path to the c++ file
    :param path_mesh: path to the downloadedd low-res mesh
    :param path_landscape: path to the captured landscape mesh
    :param config_file: path to the config .txt file for parameters
    :param output_dir: path to the output directory

    :return:
    """
    # Check for linux system
    if sys.platform == 'linux':
        # Call the c++ file via the command line
        os.system(f"{path_exec} {path_mesh} {path_landscape} {config_file} {dir_output}")
    return