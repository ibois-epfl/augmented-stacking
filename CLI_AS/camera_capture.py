"""
The module opens the camera capture a point cloud and:
- mesh the point cloud and give back a water-tight mesh
"""

import copy


import sys

from tomlkit import key
if sys.version_info[0] == 2:  # the tkinter library changed it's name from Python 2 to 3.
    import Tkinter
    tkinter = Tkinter #I decided to use a library reference to avoid potential naming conflicts with people's programs.
else:
    import tkinter
from PIL import Image
from PIL import ImageTk

import pymeshlab # keep on top as first import (why?)
import pyzed.sl as sl
import numpy as np
import open3d as o3d
import tifffile
from sklearn.cluster import KMeans
import threading
## Imports for function: convert_roit_meter_pixel
import os
import yaml 
from util import terminal

from util import visualizer
import distance_map
sys.path.append('/usr/local/lib/python3.8/dist-packages')
import cv2

#TODO: set wall scanning 1.5 x 0.7 m dimension area
ROI = [0.7,1.5] 

CENTER = [250,750]
# CENTER = [360,680]


NUMBER_OF_AVERAGE_FRAMES = 1

IS_DOWNSAMPLED = True


def rotationMatrix(r):
    """
    Simple 3D Matrix rotation function, obtained from following sources:
    https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula

    Args: 
        -r: a rotation vector, with rotation value in x, y and z direction.
    """
    ## Parameter for the rotationmatrix function
    rotationAngleDegThreshold = 0.00001

    # its length is the rotation angle
    rotationAngleDeg = np.linalg.norm(r)

    if rotationAngleDeg > rotationAngleDegThreshold:
        # its direction is the rotation axis.
        rotationAxis = r / rotationAngleDeg

        # positive angle is clockwise
        K = np.array([[       0,         -rotationAxis[2],  rotationAxis[1]],
                            [ rotationAxis[2],        0,         -rotationAxis[0]],
                            [-rotationAxis[1],  rotationAxis[0],        0       ]])

        # Note the np.dot is very important.
        R = np.eye(3) + (np.sin(np.deg2rad(rotationAngleDeg)) * K) + \
            ((1.0 - np.cos(np.deg2rad(rotationAngleDeg))) * np.dot(K, K))

        tmp = np.eye(4)
        tmp[0:3, 0:3] = R
    else:
        R = np.eye(3)
    return R

def load_transformation_matrix():

    _root_file = os.path.dirname(__file__)
    if _root_file == "":
        _calib_information_path = _root_file + "calib/utils/calibration_info.yaml"
    else:
        _calib_information_path = _root_file + "/calib/utils/calibration_info.yaml"
    # Test if the file exist as it is supposed when runing calib function entirely
    if not os.path.exists(_calib_information_path):
        terminal.error_print(
            f"No Calibration Data has been found in: {_calib_information_path}"
        )
        exit()
    else:
        ## Load the transformation matrix
        # Opening YAML file
        with open(_calib_information_path) as yaml_file:
            data = yaml.load(yaml_file, Loader=yaml.FullLoader)
        # extracting information
        matrix_data = data["3D_2D_Matrix"]
        s, f, u0, v0, dX, dY, dZ, m_x, m_y, gamma, r0, r1, r2 = (
            matrix_data["s"],
            matrix_data["f"],
            matrix_data["u0"],
            matrix_data["v0"],
            matrix_data["dX"],
            matrix_data["dY"],
            matrix_data["dZ"],
            matrix_data["m_x"],
            matrix_data["m_y"],
            matrix_data["gamma"],
            matrix_data["r0"],
            matrix_data["r1"],
            matrix_data["r2"],
        )
        Rt = np.zeros((4, 4))
        R = rotationMatrix(np.array([r0, r1, r2]))
        Rt[0:3, 0:3] = R
        Rt[:, -1] = np.array([dX, dY, dZ, 1])
        K = np.array([[f*m_x, gamma, u0, 0], [0, f*m_y, v0, 0], [0, 0, 1, 0]])
        transformation_matrix = np.dot(K,Rt)/s

        return transformation_matrix

def pcd_to_2D_image(pcd):
    """
    This function is converting a open3D point cloud into a 2D projection for the projector.
    It is using the results of the calibration function.
    """

    P = load_transformation_matrix()
    # Going through the points of the point cloud
    npy_pcd = np.asarray(pcd.points)
    npy_pcd_color = np.asarray(pcd.colors)
    img = np.zeros((1080, 1920, 3),dtype=np.uint8)
    if not len(npy_pcd) == 0:
        pointsXYZ = npy_pcd[::1]
        Pixls = []
        for i, pt in enumerate(pointsXYZ):
            X,Y,Z = pt
            # print(X,Y,Z)
            # convert XYZ to pixels
            pixels = np.dot(P, np.array([[X], [Y], [Z],[1]]))
            Pixls.append(pixels)
            # print(pixels)
            if pixels[1]<1080 and pixels[0]<1920 and pixels[0]>0 and pixels[1]>0:
                img[int(pixels[1])-2:int(pixels[1])+2,
                        int(pixels[0])-2:int(pixels[0])+2,:] = np.array([1,0,1])# np.uint8(npy_pcd_color[i,:]*255)
        Pixls = np.array(Pixls)

    return img

def convert_roi_meter_pixel(roi,center):
    """
    This function is returning a rectangular Region Of Interest in pixel slices, centered in the middle of the image.
    And take as an input an array of the width and the length of the ROI in meters.

    :param roi: Array of the width and the length of the ROI in meters.
    """

    _root_file = os.path.dirname(__file__)
    if _root_file == "":
        _calib_information_path = _root_file + "calib/utils/calibration_info.yaml"
    else:
        _calib_information_path = _root_file + "/calib/utils/calibration_info.yaml"
    # Test if the file exist as it is supposed when runing calib function entirely
    if not os.path.exists(_calib_information_path):
        terminal.error_print(f"No Calibration Data has been found in: {_calib_information_path}")
        exit()
    else:
        # Opening YAML file
        with open(_calib_information_path) as yaml_file:
            data = yaml.load(yaml_file,Loader=yaml.FullLoader)
        roi_info = data["ROI_info"]
        distance_m = roi_info["Distance_m"]
        distance_px = roi_info["Distance_px"]
        convert_m_px = distance_px/distance_m
        roi_px = np.array(roi) * convert_m_px

        ## We suppose the camera used is the zed camera, with an image acquisition of 1280x720 pixels
        ## the center is (360,640)
        slice_roi = [slice(int(center[0]-roi_px[0]/2),int(center[0]+roi_px[0]/2)),
                     slice(int(center[1]-roi_px[1]/2),int(center[1]+roi_px[1]/2))]

    return slice_roi
 
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

def get_median_cloud(zed, point_cloud, medianFrames, roi_m,center):

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

    # Convert the ROI value from meters to pixels and into a slice object.
    roi_px = convert_roi_meter_pixel(roi_m,center)
    # roi_px = ROI
    # Crop the point cloud following the ROI
    stack_of_images = stack_of_images[:, roi_px[0], roi_px[1], :]

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
    np_median_pcd = get_median_cloud(zed,point_cloud,NUMBER_OF_AVERAGE_FRAMES, ROI,CENTER)

    # From point cloud to pymeshlab mesh set + downsapling
    o3d_m = np_pcd2o3d_mesh(np_median_pcd, n_target_downasample=n_target_downasample)
    
    # TODO: clean up this code ~ condense
    # Crop mesh according to ROI
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_median_pcd)
    bbox = pcd.get_axis_aligned_bounding_box()
    o3d_m = o3d_m.crop(bbox)

    # Close the camera
    close_up_zed(zed)

    return o3d_m

def get_pcd_scene(n_target_downsample, zed, point_cloud):
    """
    Main method to get point cloud

    :param n_target_downasample: target number of points to downsample cloud
    :param zed: initilaized camera and point cloud from the camera
    return: point cloud in open3d format
    """

    # Capture the average point cloud from frames
    np_median_pcd = get_median_cloud(zed,point_cloud,NUMBER_OF_AVERAGE_FRAMES, ROI, CENTER)

    # Convert numpy to o3d cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_median_pcd)

    return pcd

class Live_stream(object):
    
    def __init__(self,zed,point_cloud,merged_landscape,rock_mesh):
        self.tk = tkinter.Tk()

        self.w, self.h = self.tk.winfo_screenwidth(), self.tk.winfo_screenheight()
        self.tk.geometry("%dx%d+-50+-50" % (self.w, self.h))
        self.state = False
        self.tk.attributes('-zoomed', True)  # This just maximizes it so we can see the window. It's nothing to do with fullscreen.
        self.tk.bind('<Escape>', self._end_stream)
        self.tk.bind("<F11>", self._toggle_fullscreen)
        self.lmain = tkinter.Label(self.tk)
        self.lmain.pack()

        self.zed = zed
        self.point_cloud = point_cloud
        self.merged_landscape = merged_landscape
        self.rock_mesh = rock_mesh

    def _end_stream(self,event=None):
        # self.tk.quit()
        self.tk.quit()
        self.tk.destroy()

    def _toggle_fullscreen(self, event=None):
        self.state = not self.state  # Just toggling the boolean
        self.tk.attributes("-fullscreen", self.state)
        return "break"

    def run(self):
        self._show_frame()
        self.tk.mainloop()

    def _show_frame(self):
        self.frame = self._get_live_stream()
        self.imgtk = ImageTk.PhotoImage(image=Image.fromarray(self.frame, mode="RGB"))
        self.lmain.configure(image=self.imgtk)
        self.lmain.after(10, self._show_frame) 
        
    def _get_live_stream(self):
        # Get point cloud from camera
        pcd = get_pcd_scene(2000, self.zed, self.point_cloud)  #TODO: check param 2000
        # visualizer.viualize_wall([pcd,self.rock_mesh],"captured pcd")

        ## Crop the pcd from a column
        cropped_pcd = _column_crop(pcd,self.rock_mesh)
        # visualizer.viualize_wall([cropped_pcd],"cropped pcd")

        ## Get upper pcd of the mesh
        upper_pcd_from_mesh = _get_upper_pcd(self.rock_mesh)
        # visualizer.viualize_wall([upper_pcd_from_mesh,self.rock_mesh],"upper pcd from mesh")

        ## Get keypoints and cluster pcd from the upper_pcd_from_mesh
        list_pcd_clusters, keypoints = _get_cluster(upper_pcd_from_mesh)
        # for cluster in list_pcd_clusters:
        #     visualizer.viualize_wall([cluster],"keypoints")
        print(keypoints)

        ## Get captured pcd clusters
        captured_pcd_clusters = _crop_pcd_on_cluster(cropped_pcd,list_pcd_clusters)
        # for cluster in captured_pcd_clusters:
        #     visualizer.viualize_wall([cluster],"captured pcd cluster")

        ## Get the Z value of the captured pcd clusters
        z_values = _get_z_value_of_pcds(captured_pcd_clusters)
        print(z_values)

        ## Compute distance
        distances = np.abs(np.array(keypoints)[:,2] - z_values)
        
        print(distances)
        ## Cluster the cropped captured pcd

        # cropped_landscape_mesh = self.merged_landscape.crop(bbox)
        # cropped_landscape_mesh = self.rock_mesh

        # visualizer.viualize_wall([cropped_landscape_mesh], "TESTCROPMESH")

        # Calculate the deviation of the rock from the cloud
        # pcd_temp = distance_map.compute(mesh=self.rock_mesh, pc=pcd)

        ## WORK IN PROGRESS
        pcd_temp = o3d.geometry.PointCloud()

        dist_pcd = np.zeros((2,3), dtype=np.float64)
        center_pcd = cropped_pcd.get_center()
        center_mesh = self.rock_mesh.get_center()
        dist_pcd[0] = center_pcd
        dist_pcd[1] = center_mesh

        pcd_temp.points = o3d.utility.Vector3dVector(dist_pcd)
        c = [(1,0,1), (0,1,1)]
        pcd_temp.colors = o3d.utility.Vector3dVector(c)

        # Show rock mesh projection
        # pcd_temp = pcd_temp + self.rock_mesh.sample_points_poisson_disk(1000)
        pcd_temp = pcd_temp + cropped_pcd
        
        if pcd_temp !=None:
            # Convert 3D>2D >> image
            img = pcd_to_2D_image(pcd_temp)
        else:
            # Create error image
            img = np.ones((1080, 1920, 3),dtype=np.uint8) * np.array([255,0,255],dtype=np.uint8)
        return img

def _get_cluster(upper_pcd_of_mesh):
    # Get the points of the point cloud
    Points = np.asarray(upper_pcd_of_mesh.points)

    # Use of K-mean for detecting 3 points in the upper point cloud  
    kmeans = KMeans(n_clusters=3, random_state=0).fit(Points)

    key_points = kmeans.cluster_centers_
    pcd_labels = kmeans.labels_
    
    list_cluster = []
    for j in range(0,3):
        pcd_cluster = o3d.geometry.PointCloud()
        cluster = []
        for i,label in enumerate(pcd_labels):
            if label == j:
                cluster.append(Points[i])
        pcd_cluster.points = o3d.utility.Vector3dVector(np.array(cluster))
        list_cluster.append(pcd_cluster)

    return list_cluster, key_points
    
def _column_crop(captured_pcd,rock_mesh,scale=1.5):
    
    # Translate the mesh
    mesh_down = copy.deepcopy(rock_mesh).translate((0, 0, -10))
    mesh_up = copy.deepcopy(rock_mesh).translate((0, 0, 10))

    # Union of the two meshes
    mesh_down_up = mesh_down + mesh_up

    # Get Axis-aligned bounding box
    bbox = mesh_down_up.get_axis_aligned_bounding_box()
    bbox = bbox.scale(scale,bbox.get_center())

    crop_captured_pcd = captured_pcd.crop(bbox)
    return crop_captured_pcd

def _crop_pcd_by_occupancy(mesh,pcd):
    # Load mesh and convert to open3d.t.geometry.TriangleMesh
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    #Create the scene 
    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(mesh)

    # Compute occupancy map
    occupancy = scene.compute_occupancy(np.asarray(pcd.points, dtype=np.float32))

    cropped_pcd = o3d.geometry.PointCloud()
    outside_points = []
    for i,point in enumerate(np.asarray(pcd.points)):
        if occupancy[i] == 0:
            outside_points.append(point)

    if len(outside_points) == 0:
        cropped_pcd.points = o3d.utility.Vector3dVector(np.array([[0,0,-2]]))
    else:
        cropped_pcd.points = o3d.utility.Vector3dVector(np.array(outside_points))

    return cropped_pcd

def _get_upper_pcd(mesh):
    # Create shifted point cloud
    subsampled_mesh = mesh.sample_points_poisson_disk(1000)
    subsampled_mesh = subsampled_mesh.translate((0, 0, 0.01))
    # Crop point cloud
    cropped_pcd = _crop_pcd_by_occupancy(mesh,subsampled_mesh)
    return cropped_pcd

def _crop_pcd_on_cluster(crop_captured_pcd,pcd_from_upper_mesh_clusters):
    list_captured_pcd_clusters = []
    for cluster in pcd_from_upper_mesh_clusters:
        cropped_cluster = _column_crop(crop_captured_pcd,cluster,scale=1.0)
        list_captured_pcd_clusters.append(cropped_cluster)
    return list_captured_pcd_clusters

def _get_z_value_of_pcds(captured_pcd_clusters):
    Z_mean = []
    Z_std = []
    for cluster in captured_pcd_clusters:
        z_mean = np.mean(np.asarray(cluster.points)[:,2])
        z_std = np.std(np.asarray(cluster.points)[:,2])
        Z_mean.append(z_mean)
        Z_std.append(z_std)
    Z_value = np.asarray(Z_mean) # + 1/2*np.asarray(Z_std)**2
    return Z_value

# def crop_pcd_by_raycasting(pcd,mesh):
#     # Load mesh and convert to open3d.t.geometry.TriangleMesh
#     mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    
#     # Scale mesh
#     mesh = mesh.scale(1.5,mesh.get_center())
    
#     # Create a scene and add the triangle mesh
#     scene = o3d.t.geometry.RaycastingScene()
#     _ = scene.add_triangles(mesh)

#     # compute occupancy
#     occupancy = scene.compute_occupancy(np.asarray(pcd.points, dtype=np.float32))

#     cropped_pcd = o3d.geometry.PointCloud()
#     inside_points = []
#     colors = []
#     for i,point in enumerate(np.asarray(pcd.points)):
#         if occupancy[i] == 1:
#             inside_points.append(point)
#             colors.append([0,1,0])
#     if len(inside_points) == 0:
#         cropped_pcd.points = o3d.utility.Vector3dVector(np.array([[0,0,-2]]))
#         cropped_pcd.colors = o3d.utility.Vector3dVector([[0,1,0]])
#     else:
#         cropped_pcd.points = o3d.utility.Vector3dVector(np.array(inside_points))
#         cropped_pcd.colors = o3d.utility.Vector3dVector(colors)
#     return cropped_pcd


