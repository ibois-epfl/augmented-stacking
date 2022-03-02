import dataset_IO
import os
import open3d as o3d


_url_low_res_dir = 'https://raw.githubusercontent.com/ibois-epfl/augmented-stacking-dataset/main/stones/low_res/'
# _name_mesh = '073ac15d-8ad2-4758-9c8c-5f8339b97edb_Mesh.ply'
_name_mesh = 'bun_zipper.ply'


def main():
    
    # V - [1] Download the low-res mesh
    # >>> [2] Compute the mesh 6dof pose
    # [3] Store/save the 6dof pose locally
    # [4] Load the high-res mesh
    # [5] Calculate deviation
    # [6] Colored captured point cloud with rgb deviation values
    # [8] Pass it to the 3D-2D pipeline


    # Download the mesh file
    dataset_IO.download_github_raw_file(_url_low_res_dir, _name_mesh)
    
    # Open and display mesh with open3d
    mesh = o3d.io.read_triangle_mesh(_name_mesh)
    o3d.visualization.draw_geometries([mesh])

    # dataset_IO.delete_file(_name_mesh)


if __name__ == "__main__":
    main()