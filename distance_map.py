
import sys
import open3d as o3d
import numpy as np
import color_map
from util import terminal

def compute(mesh, pc):

    # Load mesh and convert to open3d.t.geometry.TriangleMesh
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

    # Create a scene and add the triangle mesh
    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(mesh)
    
    if len(pc.points)!=0:
        # compute distance
        # unsigned_distance = scene.compute_distance(
        #     np.asarray(pc.points, dtype=np.float32))
        ## center of point cloud
        ## center of mesh

        # center = mesh.get_center()
        # center = np.array(center.numpy())
        # print(type(center))
        # print(center)

        unsigned_distance = pc.compute_point_cloud_distance(pcd_center)

        # compute occupancy
        # occupancy = scene.compute_occupancy(np.asarray(pc.points, dtype=np.float32))

        # normalize distance to (0,1)
        if(unsigned_distance.numpy().max() <= 0):
            raise("maximal distance should be non-zero!")
            sys.exit(1)

        # normalized_distance = unsigned_distance.numpy()/unsigned_distance.numpy().max()
        normalized_distance = unsigned_distance.numpy()
        normalized_distance[normalized_distance>0.1] = 0.1
        normalized_distance = normalized_distance/0.1

        ### DEBUG Distance
        print(f"Length of distances: {normalized_distance.shape}")
        print(f"Max distance: {np.max(normalized_distance)}")
        print(f"Min distance: {np.min(normalized_distance)}")

        # create color for each point according to the distance
        start_hue = 120
        end_hue = 0
        colors = []
        for i,p in enumerate(normalized_distance):

            colors.append(color_map.transitionOfHueRange(p, start_hue, end_hue))

            # #Test with occupancy
            # if occupancy[i]==1.0:
            #     colors.append(color_map.transitionOfHueRange(p, start_hue, end_hue))
            # else:
            #     colors.append((1,0,1))
        c = o3d.utility.Vector3dVector(colors)
        pc.colors = c

        # write colored point cloud
        # o3d.io.write_spoint_cloud("./colored_pc.ply", pc)
    else:
        terminal.error_print("WARN, distance_map.py: captured croped point cloud is empty")
        pc = None
    return pc

