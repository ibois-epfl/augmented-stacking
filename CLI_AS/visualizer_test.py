
import os
import open3d as o3d
import tkinter as tk
import numpy as np
import time



def rotate_mesh(mesh) -> None:
    R = mesh.get_rotation_matrix_from_xyz((np.pi / -8, 0, 0))
    mesh = mesh.rotate(R, center=(0.,0.,0.))

def rgb_2_norm(rgb:list) -> list:
    return [val/255 for val in rgb]

def get_monitors_size() -> list:
    """
    :return the main monitor size and width + second monitor's height
    """
    root = tk.Tk()
    root.update_idletasks()
    root.attributes('-fullscreen', True)
    root.state('iconic')
    geometry = root.winfo_geometry()

    main_width = int(geometry.split('+')[0].split('x')[0])
    main_height = int(geometry.split('+')[0].split('x')[-1])
    secondary_height = int(geometry.split('+')[-1])

    root.destroy()

    return [main_width, main_height, secondary_height]

def update_built_state(name_dir : str,
                       pcd : o3d.geometry.PointCloud,
                       mesh : o3d.geometry.TriangleMesh) -> None:
    """
    Called at each loop of main while loop.
    Read the dump folder during the stacking process and store the data in a 
    list by stacking order present in filename.
    """

    # Reset containers
    dict_stones = {}
    dict_scenes = {}

    # Wait for the directory to recive files
    while(len(os.listdir(name_dir)) == 0):
        time.sleep(1)

    # Get and fit files in dict stones/scenes
    for file_name in os.listdir(name_dir):
        if file_name.endswith(".ply"):
            path_file = os.path.join(name_dir, file_name)

            mesh = o3d.io.read_triangle_mesh(path_file)
            rotate_mesh(mesh)  # to simulate axonometry

            pos = int(file_name.split('_')[0])

            if 'stone' in file_name:
                dict_stones[pos] = mesh

            elif 'scene' in file_name:
                pcd = mesh.sample_points_uniformly(number_of_points=6000)  # cvt into pcd for visualization
                pcd.paint_uniform_color(rgb_2_norm([102,255,153]))
                dict_scenes[pos] = pcd
    
    # Sort the queries by stacking order
    dict_stones = {k: dict_stones[k] for k in sorted(dict_stones)}
    dict_scenes = {k: dict_scenes[k] for k in sorted(dict_scenes)}

    # Update stones: merge all the stones
    mesh = list(dict_stones.values())[0]
    for i, item in enumerate(list(dict_stones.values())):
        if i != len(list(dict_stones.values()))-1:
            item.paint_uniform_color([1, 0.706, 0])  # prev stones: yellow
        else:
            item.paint_uniform_color([1, 0, 0])  # last stone: red
        mesh += item

    # Update scene: refresh point cloud
    pcd = list(dict_scenes.values())[-1]

    return [pcd, mesh]


def main():

    # Dir where stones are droped
    DIR_AS_BUILT_STATE = './as_built_status/20220504113604/'

    # Get recorded stones
    mesh_stones = o3d.geometry.TriangleMesh()
    pcd_scene = o3d.geometry.PointCloud()

    # Update scene geometries
    update_geo = update_built_state(name_dir=DIR_AS_BUILT_STATE,
                                    pcd=pcd_scene,
                                    mesh=mesh_stones)
    pcd_scene_up, mesh_stones_up = update_geo[0], update_geo[1]
    pcd_scene.points = pcd_scene_up.points
    pcd_scene.colors = pcd_scene_up.colors
    mesh_stones.vertices = mesh_stones_up.vertices
    mesh_stones.vertex_normals = mesh_stones_up.vertex_normals
    mesh_stones.vertex_colors = mesh_stones_up.vertex_colors
    mesh_stones.triangles = mesh_stones_up.triangles

    # Get monitor sizes and set visualizer
    vis = o3d.visualization.Visualizer()
    monitors_size = get_monitors_size()
    vis.create_window(window_name="as_built_wall",
                      width=int(monitors_size[0]/2),
                      height=monitors_size[1],
                      left=0,
                      top=monitors_size[2])

    opt = vis.get_render_option()
    rgb = [51, 153, 255]
    map_rgb = [val/255 for val in rgb]
    opt.background_color = [map_rgb[0], map_rgb[1], map_rgb[2]]
    opt.point_size = 2
    opt.light_on = True
    opt.mesh_show_wireframe = True
    
    vis.add_geometry(pcd_scene, reset_bounding_box=True)
    vis.add_geometry(mesh_stones, reset_bounding_box=False)

    while(True):

        # Update scene geometries
        update_geo = update_built_state(name_dir=DIR_AS_BUILT_STATE,
                                        pcd=pcd_scene,
                                        mesh=mesh_stones)
        pcd_scene_up, mesh_stones_up = update_geo[0], update_geo[1]
        pcd_scene.points = pcd_scene_up.points
        pcd_scene.colors = pcd_scene_up.colors
        mesh_stones.vertices = mesh_stones_up.vertices
        mesh_stones.vertex_normals = mesh_stones_up.vertex_normals
        mesh_stones.vertex_colors = mesh_stones_up.vertex_colors
        mesh_stones.triangles = mesh_stones_up.triangles

        # Refresh visualizer
        vis.update_geometry(pcd_scene)
        vis.update_geometry(mesh_stones)
        if not vis.poll_events(): break
        vis.update_renderer()
        time.sleep(1)  # set 1fps

    vis.destroy_window()


if __name__ == "__main__":
    main()