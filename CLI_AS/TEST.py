import enum
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import open3d as o3d
from camera_capture import load_transformation_matrix


class draw_image(object):
    def __init__(self):
        self.width = 1920
        self.height = 1080
        self.image = np.zeros((self.height, self.width, 3),dtype=np.uint8)
        self.pixels = []
        self.transform_3D_2D = load_transformation_matrix()
    
    def add_3D_pixel(self,x,y,z,color,size):
        xy1 = np.dot(self.transform_3D_2D, np.array([[x], [y], [z],[1]]))
        pixel = [int(xy1[1]),int(xy1[0]),color,size]
        self.pixels.append(pixel)

    def add_pcd(self,pcd,size=2):
        npy_pts = np.asarray(pcd.points)
        npy_colors = np.asarray(pcd.colors)

        if len(npy_pts) == 0:
            print("pcd is empty")
        else:
            if len(npy_colors) < len(npy_pts):
                print("Not all points of point cloud have a color, using default color: magenta")
                for _,point in enumerate(npy_pts):
                    self.add_3D_pixel(point[0],point[1],point[2],(255,0,255),size)
            else:
                for i,point in enumerate(npy_pts):
                    self.add_3D_pixel(point[0],point[1],point[2],npy_colors[i],size)
    
    def draw_pixels(self):
        for pixel in self.pixels:
            y,x,color,size = pixel
            if y > 0 and y < self.height and x > 0 and x < self.width:
                self.image[y-size:y+size,x-size:x+size,:] = color
            else:
                print("Pixel out of bounds")
    
    def get_image(self):
        return self.image

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array([0,0,-2]))

image = draw_image()
image.add_pcd(pcd)
image.add_3D_pixel(0,0,-2,(255,0,255),5)

image.draw_pixels()

image = image.get_image()

plt.imshow(image)
plt.show()