

def call_stacking_algorithm(path_to_mesh, landscape, config_file, output_dir):
    """
    Call the stacking algorithm
    """

# Definition to run c++ file from terminal
def stackingalogrithmc(path_exec, path_mesh, path_landscape, config_file, output_dir):
    """
    Call the stacking algorithm c++ file

    :param path_mesh: path to the downloadedd low-res mesh
    :param path_landscape: path to the captured landscape mesh
    :param config_file: path to the config .txt file for parameters
    :param output_dir: path to the output directory
    """
    # Call the c++ file
    os.system(f"{path_exec} {path_toMesh} {landscape} {config_file} {output_dir}")
    return