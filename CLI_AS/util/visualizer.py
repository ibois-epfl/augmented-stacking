'''
Module to deal with visualization of meshes/stacking process.
'''

import open3d as o3d

STONE_VIS_W = 600 # px
STONE_VIS_H = 400 # px

WALL_VIS_W = 700 # px
WALL_VIS_H = 700 # px

def viualize_mesh_normal(mesh, title):
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