import pyzed.sl as sl
import numpy as np
import open3d as o3d
import tifffile
import matplotlib.pyplot as plt

from util import visualizer

# open camera
# capture point cloud
# rearrange point cloud points following center of image as origin
# export point cloud as .ply format

ROI = [slice(200, 500), slice(500, 900)] # TODO: convertir ROI in pixel in meters

NUMBER_OF_AVERAGE_FRAMES = 10

IS_DOWNSAMPLED = True


def set_up_zed():

    """
    This function is setting up the zed camera for depth capture
    
    return: The initialized camera, and the zed point cloud format/host
    """

    # Set ZED params
    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, # HD720 | 1280*720
                                camera_fps=30, # available framerates: 15, 30, 60 fps
                                depth_mode=sl.DEPTH_MODE.QUALITY, # posible mods: sl.DEPTH_MODE.PERFORMANCE/.QUALITY/.ULTRA
                                coordinate_units=sl.UNIT.METER,
                                coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP, # sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP
                                sdk_verbose = True, # Enable verbose logging
                                depth_minimum_distance=0.3, # Enable capture from 30 cm
                                depth_maximum_distance=3.0 # Enable capture up to 3m
                                ) 

    # Open ZED and catch error
    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()
    camera_info = zed.get_camera_information()
    print("ZED camera opened, serial number: {0}".format(camera_info.serial_number))

    # Setting an empty point cloud
    point_cloud = sl.Mat(zed.get_camera_information().camera_resolution.width, 
                            zed.get_camera_information().camera_resolution.height,
                            sl.MAT_TYPE.F32_C4,
                            sl.MEM.CPU)

    return zed, point_cloud

def get_median_cloud(zed, point_cloud, medianFrames, ROI):

    """
    This function is giving an average value of X, Y and Z 
    obtained by a certain number of sequentialy acquired frames.
    This helps to stabilize the coordinates acquired, in case of flickering for instance.
    
    :param zed: initialized and opened zed camera
    :param point_cloud: initialized point cloud of the zed Camera
    :param medianFrames: Number of sequentialy acquired Frames for the average value generation
    :param components: List of values 0,1 or 2 for respectively X,Y and Z coordinates.
    
    return: The median point clouds xyz (no RGB) of the acquired frames in shape (n,3)
    """

    # Get multiple frames and 
    stack_of_images = []
    for n in range(medianFrames):
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,sl.MEM.CPU, zed.get_camera_information().camera_resolution)
            point_cloud_np = point_cloud.get_data()
            stack_of_images.append(point_cloud_np)      
        else:
            print(":(")
            return None
    stack_of_images = np.array(stack_of_images)
    stack_of_images[not np.isfinite] = np.nan

    # Crop the point cloud following the ROI
    stack_of_images = stack_of_images[:, ROI[0], ROI[1], :]

    # Median the point clouds
    median = np.nanmedian(stack_of_images, axis=0)

    # Get rid of colors from point cloud
    median = median[:, :, :3]

    # Change shape of numpy to (n,3) for latter o3d transformation
    median = median.reshape((-1, 3))

    # Archive: Transform nan in zeros (median[np.isnan(median)] = 0)
    # Remove nan values from cloud
    median = median[~np.isnan(median).any(axis=1)]

    return median

def np2o3d(np_vector):

    """
    Convert a numpy vector of shape (n,3) to o3d point cloud format
    
    :param np_vector: numpy vector of shape (n,3)
    
    return: The point cloud in open3d format
    """

    # Convert numpy in o3d format
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_vector)

    return pcd

def pcd2mesh(pcd):

    """
    Process a point cloud to obtain a mesh
    
    :param pcd: point cloud to mesh
    
    return: A mesh and a clean, voxelized point cloud
    """

    # alpha = 0.005
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

    # alpha = 0.01
    # print(f"alpha={alpha:.3f}")
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

    # # Ball pivoting VERSION 1
    # pcd.estimate_normals(
    # search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=20))
    # radii = [0.005, 0.2, 0.2, 0.2] # original [0.005, 0.01, 0.02, 0.04]
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #     pcd, o3d.utility.DoubleVector(radii))


    # # Ball pivoting VERSION 2
    # pcd.estimate_normals(
    # search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)) # r=0.03

    # # estimate radius for rolling ball
    # distances = pcd.compute_nearest_neighbor_distance()
    # avg_dist = np.mean(distances)
    # radius = 0.7 * avg_dist   

    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #         pcd,
    #         o3d.utility.DoubleVector([radius, radius * 2]))


    # Poisson VERSION 1

    pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=20)) # r=0.05


    print('run Poisson surface reconstruction')
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=6)
    
    bbox = pcd.get_axis_aligned_bounding_box()
    mesh = mesh.crop(bbox)

    

    # print('visualize densities')
    # densities = np.asarray(densities)
    # density_colors = plt.get_cmap('plasma')(
    # (densities - densities.min()) / (densities.max() - densities.min()))
    # density_colors = density_colors[:, :3]
    # density_mesh = o3d.geometry.TriangleMesh()
    # density_mesh.vertices = mesh.vertices
    # density_mesh.triangles = mesh.triangles
    # density_mesh.triangle_normals = mesh.triangle_normals
    # density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)

    visualizer.viualize_mesh_normal(mesh, "meshed landscape")

    return mesh


# Set up the zed paramters and initialize
zed, point_cloud = set_up_zed()

# Average point cloud from frames
median_pcd = get_median_cloud(zed,point_cloud,NUMBER_OF_AVERAGE_FRAMES, ROI)

# Convert numpy in o3d format
pcd = np2o3d(median_pcd)

# Downsample pointcloud
if IS_DOWNSAMPLED: pcd_down = pcd.voxel_down_sample(voxel_size=0.002)


# Mesh the point cloud
mesh = pcd2mesh(pcd_down)

# Show point cloud
# o3d.visualization.draw_geometries([pcd_down])

# Save point cloud
# o3d.io.write_point_cloud("pclandscape.ply", pcd_down)
o3d.io.write_triangle_mesh("landscape.ply", mesh, write_ascii=True)

print("Mesh out")

# Close the camera
zed.close()

