
'''
The module implements the download and read of stone mesh/cloud data sets present in
the cloud (posibly github) OR locally if necessary
'''

import requests
import os
import sys
import tqdm


def download_github_raw_file(url, filename):
    """
    Download a file from GitHub, don't forget to put the raw http of github file
    e.g. https://raw.githubusercontent.com/ibois-epfl/augmented-stacking-dataset/main/stones/low_res/bun_zipper.ply

    :param url: url of the file
    :param filename: name of the file
    :return:

    """

    # Check if the file is already downloaded
    if os.path.isfile(filename):
        print(f"File {filename} already downloaded")
        return

    # Download the file and write .ply file
    try:
        url_path = os.path.join(url, filename)
        with tqdm.tqdm(unit='A', unit_scale=True, miniters=1, desc=filename) as t:
            r = requests.get(url_path, stream=True)
            # Total size in bytes.
            total_size = int(r.headers.get('content-length', 0))
            block_size = 1024
            wrote = 0
            with open(filename, 'wb') as f:
                for data in r.iter_content(block_size):
                    wrote = wrote + len(data)
                    f.write(data)
                    t.update(len(data))
    except Exception as e:
        print(f"Error while downloading {filename}")
        print(e)
        sys.exit(1)
    
    return



# Erase mesh file if exists
def delete_file(filename):
    """
    Erase a particular file

    :param url: filename path
    :return:

    """
    if os.path.isfile(filename):
        os.remove(filename)
