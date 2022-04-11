import enum
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import open3d as o3d
from camera_capture import load_transformation_matrix
import cv2

class draw_image(object):
    """   
    This class is creating an object which will allow us to list a certain number of pixels,
    with different caracteristiques, that we can at the end get into a 2D image.
    """

    def __init__(self):
        self.width = 1920
        self.height = 1080
        self.image = np.zeros((self.height, self.width, 3),dtype=np.uint8)
        self.pixels = []
        self.transform_3D_2D = load_transformation_matrix()
    
    def add_3D_pixel(self,x,y,z,color,size):
        xy1 = np.dot(self.transform_3D_2D, np.array([[x], [y], [z],[1]]))
        pixel = [int(xy1[1]),int(xy1[0]),color,size]
        i,j = pixel[:2]
        if i > 0 and i < self.height and j > 0 and j < self.width:
            self.pixels.append(pixel)
        # else:
        #     print("pixel out of bounds")

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
    
    def create_hull(self,color,size):
        if len(self.pixels) < 3:
            print("Not enough points to create hull")
            exit()
        else:
            YX = np.asarray(self.pixels,dtype=object)[:,:2]
            self.hull = ConvexHull(YX)
            for simplex in self.hull.simplices:
                cv2.line(self.image,(self.pixels[simplex[0]][:2][1],self.pixels[simplex[0]][:2][0]),(self.pixels[simplex[1]][:2][1],self.pixels[simplex[1]][:2][0]),color,size)
        
    
    def draw_pixels(self):
        for pixel in self.pixels:
            y,x,color,size = pixel
            self.image[y-size:y+size,x-size:x+size,:] = color
    
    def get_image(self):
        return self.image


## How to use it 
pcd = o3d.geometry.PointCloud()
pts = np.random.randn(1000,3)
pcd.points = o3d.utility.Vector3dVector(np.array(pts))

image = draw_image()
image.add_pcd(pcd,size=5)
image.add_3D_pixel(0.5,0.5,-1,(0,0,255),10)

image.create_hull((255,0,0),2)
image.draw_pixels()

image = image.get_image()

plt.imshow(image)
plt.show()