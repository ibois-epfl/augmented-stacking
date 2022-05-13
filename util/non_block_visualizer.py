#!/usr/bin/env python

import os, glob
import open3d as o3d
import tkinter as tk
import numpy as np
import time
import sys


class AsBuiltVisualizer():

    def __init__(self, main_dir) -> None:
        self.mesh_stones = o3d.geometry.TriangleMesh()
        self.pcd_scene = o3d.geometry.PointCloud()
        self.main_dir = main_dir
        self._dump_dir = None
        self.fps = 1

        self.nb_visualizer = o3d.visualization.Visualizer()
        self.update_geometries()
        self._set_nb_visualizer()
    
    def __del__(self) -> None:
        self.nb_visualizer.destroy_window()

    
    def update_geometries(self) -> None:
        """
        Read the dump folder and store the data and update the geometries in the
        non-blocking visualizer.
        The refresher will wait for new objects in loop if none
        """

        # Reset containers
        dict_stones = {}
        dict_scenes = {}

        # Wait for the directory to recive files
        while(len(os.listdir(self.dump_dir)) == 0):
            time.sleep(1)

        # Get and fit files in dict stones/scenes
        for file_name in os.listdir(self.dump_dir):
            if file_name.endswith(".ply"):
                path_file = os.path.join(self.dump_dir, file_name)

                mesh = o3d.io.read_triangle_mesh(path_file)
                self._rotate_mesh(mesh)  # to simulate axonometry

                pos = int(file_name.split('_')[0])

                if 'stone' in file_name:
                    dict_stones[pos] = mesh

                elif 'scene' in file_name:
                    pcd = mesh.sample_points_uniformly(number_of_points=6000)  # cvt into pcd for visualization
                    pcd.paint_uniform_color(self._rgb_2_norm([102,255,153]))
                    dict_scenes[pos] = pcd
        
        # Sort the queries by stacking order
        dict_stones = {k: dict_stones[k] for k in sorted(dict_stones)}
        dict_scenes = {k: dict_scenes[k] for k in sorted(dict_scenes)}

        # Update stones: merge all the stones
        if len(list(dict_stones.values())) != 0:
            mesh = list(dict_stones.values())[0]
            for i, item in enumerate(list(dict_stones.values())):
                if i != len(list(dict_stones.values()))-1:
                    item.paint_uniform_color([1, 0.706, 0])  # prev stones: yellow
                else:
                    item.paint_uniform_color([1, 0, 0])  # last stone: red
                mesh += item

        # Update scene: refresh point cloud
        pcd = list(dict_scenes.values())[-1]

        # Replace values in geometries
        self.pcd_scene.points = pcd.points
        self.pcd_scene.colors = pcd.colors
        self.mesh_stones.vertices = mesh.vertices
        self.mesh_stones.vertex_normals = mesh.vertex_normals
        self.mesh_stones.vertex_colors = mesh.vertex_colors
        self.mesh_stones.triangles = mesh.triangles

    def run(self) -> None:
        """
        The main rendering loop
        """
        while(True):
            self.update_geometries()
            self._update_np_visualizer()

    def _update_np_visualizer(self) -> None:
        """
        Refresh the visualizer with the updated geometries
        """
        # Refresh visualizer
        self.nb_visualizer.update_geometry(self.pcd_scene)
        self.nb_visualizer.update_geometry(self.mesh_stones)
        if not self.nb_visualizer.poll_events(): sys.exit() #TODO: or exit thread
        self.nb_visualizer.update_renderer()
        time.sleep(self.fps)
        
    def _set_nb_visualizer(self) -> None:
            """
            Instantiate the non-blocking visualizer from Open3D
            """
            # Get monitor sizes and set visualizer
            self.nb_visualizer = o3d.visualization.Visualizer()
            monitors_size = self._get_monitors_size()
            self.nb_visualizer.create_window(window_name="as_built_wall",
                            width=int(monitors_size[0]/2),
                            height=monitors_size[1],
                            left=0,
                            top=monitors_size[2])

            opt = self.nb_visualizer.get_render_option()
            rgb = [51, 153, 255]
            map_rgb = [val/255 for val in rgb]
            opt.background_color = [map_rgb[0], map_rgb[1], map_rgb[2]]
            opt.point_size = 2
            opt.light_on = True
            opt.mesh_show_wireframe = True
            
            self.nb_visualizer.add_geometry(self.pcd_scene, reset_bounding_box=True)
            self.nb_visualizer.add_geometry(self.mesh_stones, reset_bounding_box=False)
    
    def _rotate_mesh(self, mesh) -> None:
        """
        The rotation is only to simulate an axonometric view in the visualizer
        """
        R_1 = mesh.get_rotation_matrix_from_xyz((np.pi / -4, 0, np.pi))
        mesh = mesh.rotate(R_1, center=(0.,0.,0.))
    
    def _rgb_2_norm(self, rgb:list) -> list:
        return [val/255 for val in rgb]
    
    def _get_monitors_size(self) -> list:
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

    @property
    def dump_dir(self):
        self._dump_dir = max(glob.glob(os.path.join(self.main_dir, '*/')), key=os.path.getmtime)
        return self._dump_dir


def main():

    # The same as in augmented_stacking.py
    DIR_AS_BUILT_STATE = './as_built_status/'

    # Run the non-blocking visualizer
    as_built_visualizer = AsBuiltVisualizer(main_dir=DIR_AS_BUILT_STATE)
    as_built_visualizer.run()


if __name__ == "__main__":
    main()