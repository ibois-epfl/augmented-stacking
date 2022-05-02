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
from scipy.spatial import ConvexHull
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

# Number of frames taken for the point cloud acquisition.
NUMBER_OF_AVERAGE_FRAMES = 1



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
    _calib_information_path = os.path.join(_root_file, "calib/utils/calibration_info.yaml")

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

def convert_roi_meter_pixel(roi,center):
    """
    This function is returning a rectangular Region Of Interest in pixel slices, centered in the middle of the image.
    And take as an input an array of the width and the length of the ROI in meters.

    :param roi: Array of the width and the length of the ROI in meters.
        center: center of the image in pixel.
    """

    _root_file = os.path.dirname(__file__)
    _calib_information_path = os.path.join(_root_file, "calib/utils/calibration_info.yaml")

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

    """
    This is the class creating the tkinter window with the live stream of the position of the stone.
    """
    
    def __init__(self,Live_3D_space,image_drawer):
        self.tk = tkinter.Tk()

        self.w, self.h = self.tk.winfo_screenwidth(), self.tk.winfo_screenheight()
        self.tk.geometry("%dx%d+-50+-50" % (self.w, self.h))
        self.state = False
        self.tk.attributes('-zoomed', True)  # This just maximizes it so we can see the window. It's nothing to do with fullscreen.
        self.tk.bind('<Escape>', self._end_stream)
        self.tk.attributes("-fullscreen", True)
        self.lmain = tkinter.Label(self.tk)
        self.lmain.pack()

        self.Live_3D_space = Live_3D_space
        self.image_drawer = image_drawer
  
    def _end_stream(self,event=None):
        """
        Function to end the stream, linked with the escape key in __init__.
        """
        self.tk.quit()
        self.tk.destroy()

    def _toggle_fullscreen(self, event=None):
        """
        Function to toggle fullscreen, linked with the F11 key in __init__.
        """
        self.state = not self.state  # Just toggling the boolean
        self.tk.attributes("-fullscreen", self.state)

    def _show_frame(self):
        """
        Function which is called in the run function, whihch is the loop of tkinter.
        It updates the tkinter image with the acquired live stream image.
        """
        self.frame = self._get_live_stream()
        self.imgtk = ImageTk.PhotoImage(image=Image.fromarray(self.frame, mode="RGB"))
        self.lmain.configure(image=self.imgtk)
        self.lmain.after(10, self._show_frame) 
        
    def _get_live_stream(self):
        """
        Function which updates the new image, by getting an update of the 3D Space.
        This function is using the class Live_3D_space.
        """
        # Update the 3D space, with new capture points and all the distance measures
        self.Live_3D_space.update_3D_space()

        # Draw the new image for live stream
        img = self.image_drawer.draw_image_from_3D_space(self.Live_3D_space)
       
        return img

    def run(self):
        self._show_frame()
        self.tk.mainloop()

class Live_3D_space(object):
    """
    This class is containing the 3D space, where the acquired pcd is processed.
    It allows us to process the convex hull once and then update the pcd distances.
    """
    def __init__(self,rock_mesh,zed,point_cloud):
        
        self.point_cloud = point_cloud
        self.rock_mesh = rock_mesh
        self.zed = zed

        self.upper_pcd_from_mesh = self._get_upper_pcd()
        self.list_mesh_cluster, self.key_points = self._get_mesh_cluster()
        
    def _get_upper_pcd(self):
        """
        This function returns the upper pcd from the rock_mesh.
        """
        # Create shifted point cloud
        mesh  = copy.deepcopy(self.rock_mesh)
        subsampled_mesh = mesh.sample_points_poisson_disk(1000)
        subsampled_mesh = subsampled_mesh.translate((0, 0, 0.01))
        
        # Crop point cloud
        cropped_pcd = self._crop_pcd_by_occupancy(mesh.scale(1.1,mesh.get_center()),subsampled_mesh)
        return cropped_pcd
    
    def _crop_pcd_by_occupancy(self,mesh,pcd):
        """
        This function is returning a cropped point cloud.
        It will return the inverse of a crop of the pcd, using the mesh as the bounding box.
        If the points are inside the mesh, they will be removed.
        """
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

    def _get_mesh_cluster(self):
        """
        This function returns both the clusters and the centers of the clusters, of the rock mesh.
        Those centers are our fixed keypoints.
        This function is using the K-mean algorithm, which give random results.
        """
        # Get the points of the point cloud
        Points = np.asarray(self.upper_pcd_from_mesh.points)

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

    def _column_crop(self,captured_pcd,mesh,scale=1.5):
        """
        This function is returning a cropped point cloud, using as a bounding box, 
        the boundig box of the mesh, scaled with a given scale, and tranlsated along z axis.
        """
        # Translate the mesh
        mesh_down = copy.deepcopy(mesh).translate((0, 0, -10))
        mesh_up = copy.deepcopy(mesh).translate((0, 0, 10))

        # Union of the two meshes
        mesh_down_up = mesh_down + mesh_up

        # Get Axis-aligned bounding box
        bbox = mesh_down_up.get_axis_aligned_bounding_box()
        bbox = bbox.scale(scale,bbox.get_center())

        crop_captured_pcd = captured_pcd.crop(bbox)
        return crop_captured_pcd

    def _crop_pcd_on_cluster(self,pcd,list_of_mesh):
        """
        This function is returing a list of cropped points, and the centers of all the cropped point clouds.
        Each cropped point cloud is cropped using a given mesh.
        """
        list_pcds = []
        centers = []
        for mesh in list_of_mesh:
            cropped_cluster = self._column_crop(pcd,mesh,scale=0.7)
            list_pcds.append(cropped_cluster)
            center = cropped_cluster.get_center()
            centers.append(center)
        return list_pcds,np.array(centers)

    # def _get_z_value_of_pcds(self,list_pcds):
    #     """
    #     This function returns the distance along z axis, between the center of the point cloud and the keypoints.
    #     """
    #     Z_mean = []
    #     Z_std = []
    #     for pcd in list_pcds:
    #         if not len(np.asarray(pcd.points))==0:
    #             z_mean = np.mean(np.asarray(pcd.points)[:,2])
    #             z_std = np.std(np.asarray(pcd.points)[:,2])
    #             Z_mean.append(z_mean)
    #             Z_std.append(z_std)
    #         else:
    #             Z_mean.append(0)
    #             Z_std.append(0)                
    #     Z_value = np.asarray(Z_mean) # + 1/2*np.asarray(Z_std)**2
    #     return Z_value

    ## Getters
    def get_list_mesh_cluster(self):
        return self.list_mesh_cluster
    
    def get_upper_pcd(self):
        return self.upper_pcd_from_mesh

    def get_distances(self):
        return self.distances

    def get_centers(self):
        return self.centers
    
    def get_key_points(self):
        return self.key_points
    
    def update_3D_space(self):

        # Get point cloud from camera
        pcd = get_pcd_scene(2000, self.zed, self.point_cloud)  #TODO: check param 2000

        ## Crop the pcd from a column
        cropped_pcd = self._column_crop(pcd,self.rock_mesh,scale=1)

        ## Get keypoints and cluster pcd from the upper_pcd_from_mesh
        list_mesh_clusters = self.get_list_mesh_cluster()
        keypoints = self.get_key_points()

        ## Get captured pcd clusters
        captured_pcd_clusters,self.centers = self._crop_pcd_on_cluster(cropped_pcd,list_mesh_clusters)
        
        ## Get the Z value of the captured pcd clusters
        # z_values = self._get_z_value_of_pcds(captured_pcd_clusters)

        ## Compute distance
        distances = (np.array(keypoints)[:,2] - self.centers[:,2])*1000 # To convert in milimeters
        # clip the distances
        for i,distance in enumerate(distances):
            if np.abs(distance) < 5:
                distances[i] = np.sign(distance)*5
            if np.abs(distance) > 400:
                distances[i] = np.sign(distance)*400
        self.distances = distances

    
class Image_drawer(object):
    """   
    This class is creating an object which will allow us to list a certain number of pixels,
    with different caracteristiques, that we can at the end get into a 2D image.
    """

    def __init__(self,Live_3D_space):
        self.width = 1920
        self.height = 1080
        self.image = np.zeros((self.height, self.width, 3),dtype=np.uint8)
        self.pixels = []
        self.transform_3D_2D = load_transformation_matrix()
        self.Live_3D_space = Live_3D_space

    def _3D_to_2D(self,x,y,z):
        """
        This function is transforming a 3D point into a 2D point.
        """
        
        point_2D = np.dot(self.transform_3D_2D, np.array([[x], [y], [z],[1]]))
        point_2D = point_2D[0:2]
        return point_2D
        
    def _add_3D_point_to_image(self,x,y,z,color,size):
        """
        This function is taking as an input x,y,z coordinates from a point in space, 
        and caracteristiques of the pixel, like color and size.
        And if the coordinate is in the image range, we add the pixel to the list of pixels.
        """
        if not np.isnan(x) and not np.isnan(y) and not np.isnan(z):
            pixel_coord = self._3D_to_2D(x,y,z)
            pixel = [int(pixel_coord[0][0]),int(pixel_coord[1][0]),color,size]
            i,j = pixel_coord
            if i > 0 and i < self.height and j > 0 and j < self.width:
                self.pixels.append(pixel)
            else:
                print(f"X,Y,Z: {x},{y},{z}, giving Pixel: {i}, {j} are out of bounds for image of size {self.height}, {self.width}")
        else:
            print(f"point: [{x},{y},{z}] is not admissible")
 
    def _add_pcd_to_image(self,pcd,size=2,color=[255,0,255]):

        """
        This function is adding an entire point cloud to the image.
        It takes as an input an o3d point cloud, and the caracteristiques of the pixel, like color and size.
        """
        npy_pts = np.asarray(pcd.points)
        npy_colors = np.asarray(pcd.colors)

        if len(npy_pts) == 0:
            print("pcd is empty")
        else:
            if len(npy_colors) < len(npy_pts):
                for _,point in enumerate(npy_pts):
                    self._add_3D_point_to_image(point[0],point[1],point[2],color,size)
            else:
                for i,point in enumerate(npy_pts):
                    self._add_3D_point_to_image(point[0],point[1],point[2],npy_colors[i],size)
    
    def _draw_convex_hull_on_image(self,color,size):
        """
        This function is creating a convex hull out of all the pixels added in the pixels list.
        It will draw the convex hull on the image using cv2.line.
        """
        if len(self.pixels) < 3:
            print("Not enough points to create hull")
            exit()
        else:
            YX = np.asarray(self.pixels,dtype=object)[:,:2]
            self.hull = ConvexHull(YX)
            for simplex in self.hull.simplices:
                cv2.line(self.image,(self.pixels[simplex[0]][:2][1],self.pixels[simplex[0]][:2][0]),(self.pixels[simplex[1]][:2][1],self.pixels[simplex[1]][:2][0]),color,size)
        
    def _draw_pixels(self):
        """
        This function is drawing all the pixels declared in pixel list on the image.
        """
        for pixel in self.pixels:
            i,j,color,size = pixel
            self.image[i-size:i+size,j-size:j+size,:] = color
    
    def _empty_pixels(self):
        """
        This function is emptying the pixels list.
        """
        self.pixels = []
    
    def _mm_2_pxl(self,distance):
        """
        This function is converting the distance in milimeters to pixels.
        It is doing a linear transformation, with a slope of:
            a = (MAX_pxl_length-min_pxl_length)/(MAX_mm_length - min_mm_length)
        """
        ## PARAMS:
        min_pxl_length = 5
        MAX_pxl_length = 50
        min_mm_length = 5
        MAX_mm_length = 400

        a = (MAX_pxl_length-min_pxl_length)/(MAX_mm_length - min_mm_length)
        b = min_pxl_length -a*min_mm_length
        return a*distance +b

    def clear_image(self):
        """
        This funcion is setting the image to black.
        """
        self.image = np.zeros((self.height, self.width, 3),dtype=np.uint8) 

    def draw_image_from_3D_space(self,Live_3D_space):
        """
        This function is drawing the image from the 3D space.
        """
        # Taking the updated version of the 3D space
        self.Live_3D_space = Live_3D_space
        # Clearing all old pixels
        self._empty_pixels()
        # Empty the image
        self.clear_image()
        # Drawing the convex hull
        upper_pcd = self.Live_3D_space.get_upper_pcd()
        self._add_pcd_to_image(upper_pcd)
        self._draw_convex_hull_on_image(color=[0,255,0],size=4)
        # Removing the points used to create the convex hull
        self._empty_pixels()

        keypoints = self.Live_3D_space.get_key_points()
        distances = self.Live_3D_space.get_distances()

        ## Add points to image
        for i,distance in enumerate(distances):
            radius = self._mm_2_pxl(np.abs(distance))
            # Adding points from point cloud with updated distance
            if distance > 0:
                self._add_3D_point_to_image(keypoints[i][0],keypoints[i][1],keypoints[i][2],(255,0,0),int(radius))
            else:
                self._add_3D_point_to_image(keypoints[i][0],keypoints[i][1],keypoints[i][2],(0,0,255),int(radius))

            # Adding points from keypoints
            self._add_3D_point_to_image(keypoints[i][0],keypoints[i][1],keypoints[i][2],(255,255,255),5)
        self._draw_pixels()
        return self.image

    def get_image(self):
        return self.image

