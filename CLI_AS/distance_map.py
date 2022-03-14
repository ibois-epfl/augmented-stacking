
import sys
import open3d as o3d
import numpy as np
import color_map


def compute(mesh, pc):

    # Load mesh and convert to open3d.t.geometry.TriangleMesh
    # mesh = o3d.io.read_triangle_mesh("./foundation.ply")

    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

    # Create a scene and add the triangle mesh
    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(mesh)

    # Load point clound to be adjusted
    # pc = o3d.io.read_point_cloud("./A46_point_cloud.ply")
    #R = pc.get_rotation_matrix_from_xyz((np.pi / 6, 0, np.pi / 6))
    # pc.rotate(R)

    # compute distance
    unsigned_distance = scene.compute_distance(
        np.asarray(pc.points, dtype=np.float32))

    # normalize distance to (0,1)
    if(unsigned_distance.numpy().max() <= 0):
        raise("maximal distance should be non-zero!")
        sys.exit(1)

    normalized_distance = unsigned_distance.numpy()/unsigned_distance.numpy().max()

    # create color for each point according to the distance
    start_hue = 120
    end_hue = 0
    colors = []
    for p in normalized_distance:
        colors.append(color_map.transitionOfHueRange(p, start_hue, end_hue))
    c = o3d.utility.Vector3dVector(colors)
    pc.colors = c

    # write colored point cloud
    # o3d.io.write_point_cloud("./colored_pc.ply", pc)

    return pc
