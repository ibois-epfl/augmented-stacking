"""
The module opens the camera capture a point cloud and:
- mesh the point cloud and give back a water-tight mesh
"""

import pymeshlab # keep on top as first import (why?)
import pyzed.sl as sl
import numpy as np
import open3d as o3d
import tifffile
import matplotlib.pyplot as plt
import sys

from util import visualizer


#TODO: set wall scanning 1.5 x 0.7 m dimension area
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

def close_up_zed(zed_cam):
    """
    If zed it is open it closes the camera.

    :param zed_cam: the camera zed to close
    """
    zed_cam.close()


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

    """
    Convert a numpy vector of shape (n,3) to o3d point cloud format
    
    :param np_vector: numpy vector of shape (n,3) of the point cloud
    
    return: The point cloud in open3d format
    """
    # Convert numpy in o3d format
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_vector)

    return pcd

def np_pcd2o3d_mesh(np_pcd, n_target_downasample=None):
    """
    Mesh point cloud in format numpy in mesh format open3d.
    If the downsample parameter is input it downsize the cloud before
    meshing. Meshing and downsampling is done with pymeshlab, which offers
    a clean, water-tight meshing method.
    !!! No COLORS !!!

    :param np_pcd: point cloud in format numpy vector (n,3)
    :param n_target_downsample: int of target points after point cloud
    unifrom downsample

    return: o3d mesh
    """

    # Create a new pymeshlab mesh and meshset
    pyml_m_pcd = pymeshlab.Mesh(np_pcd)
    pyml_ms = pymeshlab.MeshSet()
    pyml_ms.add_mesh(pyml_m_pcd)

    # Downsample the cloud
    if (n_target_downasample is None):
        pyml_ms.generate_simplified_point_cloud(samplenum=0)
    else:
        if (isinstance(n_target_downasample, int)):
            pyml_ms.generate_simplified_point_cloud(samplenum=n_target_downasample)
        else:
            print("The target for the downsample should be an int")
            exit()

    # Compute normals and mesh the point cloud
    pyml_ms.compute_normal_for_point_clouds(flipflag=True,viewpos=[0,0,0])
    pyml_ms.generate_surface_reconstruction_screened_poisson(preclean=True)

    # Return the mesh from the dataset
    try:
        pyml_m = pyml_ms.current_mesh()
    except:
        print("Error!", sys.exc_info()[0], "occurred.")
        sys.exit("The pymeshlab MeshSet does not contain any active mesh")
    
    # Convert from pyml mesh to o3d mesh (n.b.: colors set to 0,0,0)
    pyml_vertices = pyml_m.vertex_matrix().astype(np.float64)
    pyml_vertices_normals = pyml_m.vertex_normal_matrix().astype(np.float64)

    pyml_faces = pyml_m.face_matrix()
    pyml_faces_normals = pyml_m.face_normal_matrix().astype(np.float64)

    print(f'pyml mesh\n',

          f'vertices shape: {pyml_vertices.shape}\n',
          f'vertices dtype: {pyml_vertices.dtype}\n',
          f'vertices normals shape: {pyml_vertices_normals.shape}\n',
          f'vertices normals dtype: {pyml_vertices_normals.dtype}\n',

          f'faces shape: {pyml_faces.shape}\n',
          f'faces dtype: {pyml_faces.dtype}\n',
          f'faces normals shape: {pyml_faces_normals.shape}\n',
          f'faces normals dtype: {pyml_faces_normals.dtype}\n')

    o3d_m = o3d.geometry.TriangleMesh()

    o3d_m.vertices = o3d.utility.Vector3dVector(pyml_vertices)
    o3d_m_vertices = np.asarray(o3d_m.vertices)
    o3d_m.vertex_normals = o3d.utility.Vector3dVector(pyml_vertices_normals)
    o3d_m_vertex_normals = np.asarray(o3d_m.vertex_normals)
    o3d_m.vertex_colors = o3d.utility.Vector3dVector(np.zeros(pyml_vertices.shape))
    o3d_m_vertex_clr = np.asarray(o3d_m.vertex_colors)

    o3d_m.triangles = o3d.utility.Vector3iVector(pyml_faces)
    o3d_m_triangles = np.asarray(o3d_m.triangles)
    o3d_m.triangle_normals = o3d.utility.Vector3dVector(pyml_faces_normals)
    o3d_m_triangles_normals = np.asarray(o3d_m.triangle_normals)

    print(f'o3d mesh:\n',

          f'vertices shape: {o3d_m_vertices.shape}\n',
          f'vertices dtype: {o3d_m_vertices.dtype}\n',
          f'vertices normals shape: {o3d_m_vertex_normals.shape}\n',
          f'vertices normals dtype: {o3d_m_vertex_normals.dtype}\n',
          f'vertices colors shape: {o3d_m_vertex_clr.shape}\n',
          f'vertices colors dtype: {o3d_m_vertex_clr.dtype}\n',
          
          f'triangles shape: {o3d_m_triangles.shape}\n',
          f'triangles dtype: {o3d_m_triangles.dtype}\n',
          f'triangles normals shape: {o3d_m_triangles_normals.shape}\n',
          f'triangles normals dtype: {o3d_m_triangles_normals.dtype}\n')
    
    # Check the sanity of the mesh
    err_msg = 'ERROR:WrongMeshConvertion: The mesh convert between pymeshlab and open3d is wrong.'
    assert len(o3d_m_vertices) == len(pyml_vertices), err_msg
    assert len(o3d_m_vertex_normals) == len(pyml_vertices_normals), err_msg
    assert len(o3d_m_triangles) == len(pyml_faces), err_msg

    return o3d_m


def get_mesh_scene(n_target_downasample):
    """
    Main method to get point cloud and mesh

    :param n_target_downasample: target number of points to downsample cloud

    return: mesh in open3d format
    """
    # Set up the zed parameters and initialize
    zed, point_cloud = set_up_zed()

    # Average point cloud from frames
    np_median_pcd = get_median_cloud(zed,point_cloud,NUMBER_OF_AVERAGE_FRAMES, ROI)

    # From point cloud to pymeshlab mesh set + downsapling
    o3d_m = np_pcd2o3d_mesh(np_median_pcd, n_target_downasample=n_target_downasample)

    # Close the camera
    close_up_zed(zed)

    return o3d_m
