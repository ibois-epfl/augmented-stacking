import numpy as np
import matplotlib.pyplot as plt



def draw_grid(nb_lines_X=3,nb_lines_Y=3,line_width=4):

    shape=(720,1080,3)
    Img = np.ones(shape,dtype=np.float32)

    X_space = (shape[0] - nb_lines_X*line_width)//(nb_lines_X+1)
    Y_space = (shape[1] - nb_lines_Y*line_width)//(nb_lines_Y+1)
    print(X_space)

    for i in range(1,nb_lines_X+1):
        Img[i*X_space:i*X_space+line_width+1,:,1:]=0
    for i in range(1,nb_lines_Y+1):
        Img[:,i*Y_space:i*Y_space+line_width+1,1:]=0
    return Img

grid = draw_grid()

plt.imshow(grid)
plt.show()

plt.imsave("calibration_grid.png",grid)