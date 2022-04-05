import camera_capture
import numpy as np
from util import terminal
from util import visualizer

answer = None
while not answer == 'Y':
    merged_landscape = camera_capture.get_mesh_scene(2000)
    vis = visualizer.viualize_wall_class([merged_landscape],"DEBUG")
    vis.destroy()
    del vis
    answer = terminal.user_input("Is the stone placed correctly ? (Y/n)")
