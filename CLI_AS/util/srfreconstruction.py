
import pymeshlab


def reconstruct(pc, out_dir):
    """generate mesh from point cloud

    Args:
        point cloud
        out_dir: path to output folder
    """

    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(pc)
    ms.generate_simplified_point_cloud(samplenum=2000)
    ms.compute_normal_for_point_clouds(flipflag=True,viewpos=[0,0,0])
    ms.generate_surface_reconstruction_screened_poisson(preclean=True)
    out_path = out_dir+'/landscape_reconstructed.ply'
    ms.save_current_mesh(out_path)

    return out_path


# if __name__ == '__main__':
#     reconstruct('landscape_test_bad_case.ply',
#                 './temp')
