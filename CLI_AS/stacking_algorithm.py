
'''
Moule to call EESD stacking algorithm to compute the 6dof pose of the rock and update the landscape
For now the output .txt (6dof) is written to disk in ./temp folder because we don´t have pybinding
'''

import sys
import subprocess
import colorama as cr


def compute(path_exec, path_mesh, path_landscape, config_file, name_output, dir_output):
    """
    Call the stacking algorithm c++ file

    :param path_exec: path to the c++ file
    :param path_mesh: path to the downloadedd low-res mesh
    :param path_landscape: path to the captured landscape mesh
    :param config_file: path to the config .txt file for parameters
    :param output_dir: path to the output directory

    :return: write locally a .txt file with the 6dof pose
    """
    # Call the stacking algorithm c++ via command line (Linux only)
    if sys.platform == 'linux':
        cmd = f'{path_exec} {path_mesh} {path_landscape} {config_file} {name_output} {dir_output}'
        
        try:
            cr.init()
            print(cr.Fore.CYAN)
            subprocess.check_call(cmd, shell=True)
            print(cr.Style.RESET_ALL)
        except subprocess.CalledProcessError as e:
            print('Error: Failed to call the c++ stacking algorithm file')
            print(f"ERROR MESSAGE: {e.output}")
    else:
        print("Error: Only Linux ARCH64 systems are supported")
        sys.exit(1)

    return