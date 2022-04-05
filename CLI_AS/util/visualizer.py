'''
Module to deal with visualization of meshes/stacking process.
'''

import open3d as o3d

from util import terminal

STONE_VIS_W = 600 # px
STONE_VIS_H = 400 # px

WALL_VIS_W = 700 # px
WALL_VIS_H = 700 # px

def viualize_mesh_normal(mesh, title):
    terminal.custom_print('>>> Press [Esc] to continue ...')

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title,
                      width=STONE_VIS_W,
                      height=STONE_VIS_H)
    vis.add_geometry(mesh)
    mesh.compute_vertex_normals()
    
    opt = vis.get_render_option()
    opt.mesh_color_option = o3d.visualization.MeshColorOption.Normal
    opt.background_color = [0, 0, 0]

    vis.run()
    vis.destroy_window()


def viualize_wall(geometries, title):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title,
                      width=WALL_VIS_W,
                      height=WALL_VIS_H)
    for g in geometries:
        vis.add_geometry(g)

    opt = vis.get_render_option()
    opt.background_color = [0, 0, 0]
    opt.mesh_color_option = o3d.visualization.MeshColorOption.Normal

    vis.run()
    vis.destroy_window()

## DEBUG changed function into class for destroying pruposes
class viualize_wall_class(object):
    def __init__(self,geometries, title):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name=title,
                        width=WALL_VIS_W,
                        height=WALL_VIS_H)
        for g in geometries:
            self.vis.add_geometry(g)

        opt = self.vis.get_render_option()
        opt.background_color = [0, 0, 0]
        opt.mesh_color_option = o3d.visualization.MeshColorOption.Normal

        self.vis.run()
    
    def destroy(self):
        self.vis.close()
        self.vis.destroy_window()