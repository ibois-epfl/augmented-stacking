
import open3d as o3d
import tkinter as tk
import numpy as np

def _rgb_2_norm(rgb:list) -> list:
    return [val/255 for val in rgb]


def main():

    # Post-pro geoemtry ----------------------------------------------

    # Set all the stones by addition order and color the last one in red
    mesh_stone_path = './as_built_status/20220504113604/stone_mesh_E22.ply'
    mesh_scene_path = './as_built_status/20220504113604/scene_mesh_E22.ply'

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)
    mesh_scene = o3d.io.read_triangle_mesh(mesh_scene_path)
    mesh_stone = o3d.io.read_triangle_mesh(mesh_stone_path)

    # Rotate the objects to create axo view
    R = mesh_scene.get_rotation_matrix_from_xyz((np.pi / -4, 0, 0))
    mesh_scene = mesh_scene.rotate(R, center=(0.,0.,0.))
    mesh_stone = mesh_stone.rotate(R, center=(0.,0.,0.))

    # Mesh scene to pcd scene
    pcd_scene = mesh_scene.sample_points_uniformly(number_of_points=6000)
    pcd_scene.paint_uniform_color(_rgb_2_norm([102,255,153]))



    # ----------------------------------------------

    # Get screen info to fit windows size
    root = tk.Tk()
    # screen_width = root.winfo_screenwidth()
    # screen_height = root.winfo_screenheight()
    # print(screen_width)
    # print(screen_height)
    root.update_idletasks()
    root.attributes('-fullscreen', True)
    root.state('iconic')
    geometry = root.winfo_geometry()
    root.destroy()
    print(geometry)


    # Set visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="as_built_wall",
                      width=int(1920/2),
                      height=1080,
                      left=0,
                      top=1152)  # int(screen_height/2)

    opt = vis.get_render_option()
    # rgb = [204, 153, 255]
    rgb = [51, 153, 255]
    map_rgb = [val/255 for val in rgb]
    opt.background_color = [map_rgb[0], map_rgb[1], map_rgb[2]]
    opt.point_size = 2
    
    vis.add_geometry(pcd_scene, reset_bounding_box=True)
    vis.add_geometry(mesh_stone, reset_bounding_box=False)



    while(True):

        # Update geometries
        vis.update_geometry(mesh_scene)
        vis.add_geometry(mesh_stone)

        # Refresh visualizer
        if not vis.poll_events(): break
        vis.update_renderer()

    vis.destroy_window()


if __name__ == "__main__":
    main()