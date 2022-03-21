from calibration_functions import display_calibration
import argparse

def main(calib_grid_path):
    display_calibration(calib_grid_path)


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description="Get the Calibration Points",
                                   formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        '-i', '--imagePath',
        help="Path of the image we want in full screen, here the calibration grid.",
        type=str,
        default=None
    )
    args = parser.parse_args()
    if args.imagePath == None:
        print("No image path is given...")
        exit()
    
    main(args.imagePath)
