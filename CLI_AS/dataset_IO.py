
'''
The module implements the download and read of stone mesh/cloud data sets present in
the cloud (posibly github) OR locally if necessary
'''

# Download a mesh object from git hub repository
def download_mesh(url, filename):
    import urllib.request
    import tarfile
    import os
    import shutil
    import sys
    import platform

    # Check if the file is already downloaded
    if os.path.isfile(filename):
        print("File already downloaded")
        return

    # Download the file
    try:
        print("Downloading file from: " + url)
        mesh = urllib.request.urlretrieve(url, filename)
    except:
        print("Error downloading file")
        sys.exit()
    
    print(type(mesh))
    return mesh


    # # Extract the file
    # print("Extracting file")
    # tar = tarfile.open(filename)
    # tar.extractall()
    # tar.close()

    # # Remove the tar file
    # os.remove(filename)

    # # Rename the extracted folder
    # if platform.system() == "Windows":
    #     os.rename("stone_mesh", "stone_mesh_win")
    # else:
    #     os.rename("stone_mesh", "stone_mesh_linux")

    # # Remove the extracted folder
    # shutil.rmtree("stone_mesh_linux")

def test():
    print("test")