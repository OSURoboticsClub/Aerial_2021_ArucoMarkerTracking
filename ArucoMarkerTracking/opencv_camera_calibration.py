"""
Based off of https://docs.opencv.org/3.4.15/dc/dbb/tutorial_py_calibration.html
"""

import numpy as np
import cv2
import glob
import sys

#---SET CHECKERBOARD PARAMS---#
n_rows = 6
n_cols = 9
sqr_dim = 24    # in mm

camera_folder = "./laptop_webcam"
image_type = "jpg"
#-----------------------------#

# Set cv2 Termination Criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, sqr_dim, 0.001)

# Prepare object points
obj_p = np.zeros((n_rows * n_cols, 3), np.float32)
obj_p[:, :2] = np.mgrid[0:n_cols, 0:n_rows].T.reshape(-1, 2)

# Create Arrays To Store Object/Image Points from all images
obj_points = [] # 3d point in real world space
img_points = [] #2d points in image plane

if __name__=='__main__':
    # Handle call arguments
    if len(sys.argv) < 6:
        print("\nInsufficient command line arguments provided. Using default values. For more info pass with -h for help. \n")
    else:
        camera_folder = sys.argv[1]
        image_type = sys.argv[2]
        n_rows = int(sys.argv[3])
        n_cols = int(sys.argv[4])
        sqr_dim = int(sys.argv[5])
    
    if '-h' in sys.argv or '--h' in sys.argv:
        print("\n OPENCV IMAGE CALIBRATION FROM SET OF IMAGES")
        print("\tCall: python opencv_camera_calibration.py <folder? <image_type> <n_rows> <n_cols> <square_dimension>")
        print("\nThis script creates the following files:\n\t- camera_distortion.txt\n\t- camera_matrix.txt\n\n")
        sys.exit()
    
    # Find all image files
    filename_template = camera_folder + "/*." + image_type
    images = glob.glob(filename_template)

    print("\n %d images were found\n" % (len(images)))
    if len(images) < 9:
        print("Insufficient number of images were found; provide at least 10.")
        sys.exit()
    
    n_patterns_found = 0
    distored_img = images[1] # arbitrary assignment

    for filename in images:
        if 'calibresult' in filename:
            continue
        
        # read file and convert to greyscale
        img = cv2.imread(filename)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        print("\tReading image ", filename)

        # find chess board corners
        ret, corners = cv2.findChessboardCorners(img_gray, (n_cols, n_rows), None)

        # if found
        if ret == True:
            print("Checkboard found! Press ESC to skip or ENTER to accept")
            corners2 = cv2.cornerSubPix(img_gray, corners, (11,11), (-1, -1), criteria)
            cv2.imshow('img', img)

            k = cv2.waitKey(0) & 0xFF
            if k == 27: #ESC
                print ("Image Skipped")
                distored_img = filename
                continue

            print("Image accepted")
            n_patterns_found += 1
            obj_points.append(obj_p)
            img_points.append(corners2)

        else:
            distored_img = filename

    cv2.destroyAllWindows()

    print("n_patterns_found: %d" % (n_patterns_found))
    if (n_patterns_found > 1):
        print("Found %d good images" % (n_patterns_found))
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, img_gray.shape[::-1], None, None)

        # undistort an image
        img = cv2.imread(distored_img)
        h, w = img.shape[:2]
        print("Image to undistort: ", distored_img)
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

        #undistort
        map_x, map_y = cv2.initUndistortRectifyMap(mtx, dist, None, new_camera_matrix, (w,h), 5)
        dst = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)

        # crop image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        print("ROI: ", x, y, w, h)

        cv2.imwrite(camera_folder + "/calibresult.png", dst)
        print("Calibrated picture saved as calibresult.png")
        print("Calibration Matrix: ")
        print(mtx)
        print("Disortion: ", dist)

        # Save the result
        filename = camera_folder + "/cameraMatrix.txt"
        np.savetxt(filename, mtx, delimiter=',')
        filename = camera_folder + "/cameraDistortion.txt"
        np.savetxt(filename, dist, delimiter=',')

        mean_error = 0
        for i in range(len(obj_points)):
            img_points_2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(img_points[i], img_points_2, cv2.NORM_L2) / len(img_points_2)
            mean_error += error

        print("Mean error: ", mean_error / len(obj_points))
    
    else:
        print("In order to calibrate you need at least 9 valid pictures; try again.")
