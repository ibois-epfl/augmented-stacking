import numpy as np
import os
import cv2
import time

# Set resolution for the video capture
# Function adapted from https://kirr.co/0l6qmh
def change_res(cap, width, height):
    cap.set(3, width)
    cap.set(4, height)

# Standard Video Dimensions Sizes
STD_DIMENSIONS =  {
    "480p": (640, 480),
    "720p": (1280, 720),
    "1080p": (1920, 1080),
    "4k": (3840, 2160),
}


# grab resolution dimensions and set video capture to it.
def get_dims(cap, res='1080p'):
    width, height = STD_DIMENSIONS["480p"]
    if res in STD_DIMENSIONS:
        width,height = STD_DIMENSIONS[res]
    ## change the current caputre device
    ## to the resulting resolution
    change_res(cap, width, height)
    return width, height

# Video Encoding, might require additional installs
# Types of Codes: http://www.fourcc.org/codecs.php
VIDEO_TYPE = {
    'avi': cv2.VideoWriter_fourcc(*'XVID'),
    #'mp4': cv2.VideoWriter_fourcc(*'H264'),
    'mp4': cv2.VideoWriter_fourcc(*'XVID'),
}

def get_video_type(filename):
    filename, ext = os.path.splitext(filename)
    if ext in VIDEO_TYPE:
      return  VIDEO_TYPE[ext]
    return VIDEO_TYPE['avi']

def return_camera_indexes():
    index = 0
    arr = []
    i = 10
    while i > 0:
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            arr.append(index)
            cap.release()
        index += 1
        i -= 1
    return arr

def getFolderSize(folder):
    total_size = os.path.getsize(folder)
    for item in os.listdir(folder):
        itempath = os.path.join(folder, item)
        if os.path.isfile(itempath):
            total_size += os.path.getsize(itempath)
        elif os.path.isdir(itempath):
            total_size += getFolderSize(itempath)
    return int(total_size / (1024 * 1024 * 1024))



def main():

    with open('./code/config.txt', 'r') as f:
        dir = f.readline().strip()

    res = '720p'

    for i in return_camera_indexes():
        cap = cv2.VideoCapture(i)
        width, height = get_dims(cap, res)
        while True:
            ret, frame = cap.read()
            cv2.putText(frame, str(i), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

    index = int(input("Enter the index of the direct camera you want to use: "))

    cap = cv2.VideoCapture(index)
    print(cap.get(3), cap.get(4))

    folder_name = time.strftime("%Y_%m_%d_%H_%M_%S")
    folder_path = os.path.join(dir, folder_name)
    os.mkdir(folder_path)
    filename = os.path.join(folder_path, f'{folder_name}_direct_camera.avi')

    out = cv2.VideoWriter(filename, get_video_type(filename), 25, get_dims(cap, res))

    current_size = 0
    last_size = 0
    record = False
    while True:
        ret, frame = cap.read()
        k = cv2.waitKey(1)
        show_frame = frame

        if k%256 == 32:
            record = not record

        if record:
            out.write(frame)
            cv2.putText(show_frame, "recording", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(show_frame, "NOT recording", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        current_size = getFolderSize(folder_path)
        if current_size != last_size:
            print(f"Current size: {current_size} GB")
        last_size = current_size
        
        cv2.imshow('frame',show_frame)
        if k & 0xFF == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()