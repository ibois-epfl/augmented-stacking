import tifffile
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np

file = "A46_point_cloud.tiff"

img=tifffile.imread(file)
img_vector = img.reshape((img.shape[0]*img.shape[1],3))


# img_X= img[:,:,0]
# img_Y= img[:,:,1]
# img_Z= img[:,:,2]
# plt.imshow(img_X)
# plt.show()
# plt.imshow(img_Y)
# plt.show()
# plt.imshow(img_Z)
# plt.show()

xyz = np.random.rand(100, 3)
print (xyz.shape)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(img_vector)
o3d.io.write_point_cloud("A46_point_cloud.ply", pcd)