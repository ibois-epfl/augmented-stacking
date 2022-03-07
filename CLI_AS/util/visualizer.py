'''
Module to deal with visualization of meshes/stacking process.
'''

import open3d as o3d

STONE_VIS_W = 600 # px
STONE_VIS_H = 400 # px

def viualize_mesh_normal(mesh, title):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title,
                      width=STONE_VIS_W,
                      height=STONE_VIS_H)
    vis.add_geometry(mesh)
    mesh.compute_vertex_normals()
    vis.get_render_option().mesh_color_option = o3d.visualization.MeshColorOption.Normal
    vis.run()
    vis.destroy_window()