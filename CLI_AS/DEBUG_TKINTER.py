import camera_capture
import numpy as np
from util import terminal
from util import visualizer

answer = None
while not answer == 'Y':
    merged_landscape = camera_capture.get_mesh_scene(2000)
    vis = visualizer.viualize_wall([merged_landscape],"DEBUG")
    vis.destroy()
    del vis
    zed, point_cloud = camera_capture.set_up_zed()

    Live = camera_capture.Live_stream(zed,point_cloud,merged_landscape)
    Live.run()

    answer = terminal.user_input("Is the stone placed correctly ? (Y/n)")
    zed.close()