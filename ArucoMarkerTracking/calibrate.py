""" Adapted from https://github.com/tizianofiorenzani/how_do_drones_work/tree/master/opencv """

import numpy as np
import cv2
import glob
import os

def calibrate_camera(checkboard_n_rows, 
                     checkboard_n_cols, 
                     checkboard_sqr_dim, 
                     camera_width, 
                     camera_height, 
                     image_save_path="./calibration", 
                     image_filename_base="snapshot",
                     image_type="jpg"):
    """ 
    Produces cameraMatrix and cameraDistortion matrices in .txt files for use with opencv camera operations.
    
    Instructions:
        1) Print the chessBoard.jpg without any adaption to the page

        2) Accurately measure the side of the printed chess board

        3) Mount the chess board on a rigid and flat panel

        4) Call this function. To get good results, take at least 20 different pictures varying the angles and distances.

        5) Use the output cameraMatrix.txt and cameraDistortion.txt files
    
    """

    # --- Capture Camera Calibration Images --- #

    cap = cv2.VideoCapture(0)
    if camera_width > 0 and camera_height > 0:
        print("Setting the custom Width and Height")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
    try:
        if not os.path.exists(image_save_path):
            os.makedirs(image_save_path)
            image_save_path = os.path.dirname(image_save_path)
            try:
                os.stat(image_save_path)
            except:
                os.mkdir(image_save_path)
    except:
        pass

    n_snaps = 0
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    filename = "%s/%s_%d_%d_" %(image_save_path, image_filename_base, w, h)
    while True:
        _, frame = cap.read()

        cv2.imshow('Camera Calibration', frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        if key == ord(' '):
            print("Saving image ", n_snaps)
            cv2.imwrite("%s%d.jpg"%(filename, n_snaps), frame)
            n_snaps += 1

    cap.release()
    cv2.destroyAllWindows()


    # --- Calibrate Camera --- #

    # Set cv2 Termination Criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, checkboard_sqr_dim, 0.001)

    # Prepare object points
    obj_p = np.zeros((checkboard_n_rows * checkboard_n_cols, 3), np.float32)
    obj_p[:, :2] = np.mgrid[0:checkboard_n_cols, 0:checkboard_n_rows].T.reshape(-1, 2)

    # Create Arrays To Store Object/Image Points from all images
    obj_points = [] # 3d point in real world space
    img_points = [] # 2d points in image plane
    
    # Find all image files
    filename_template = image_save_path + "/*." + image_type
    images = glob.glob(filename_template)

    if len(images) < 10:
        raise ValueError("Insufficient number of images were found; provide at least 10.")
    
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
        ret, corners = cv2.findChessboardCorners(img_gray, (checkboard_n_cols, checkboard_n_rows), None)

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

        cv2.imwrite(image_save_path + "/calibresult.png", dst)
        print("Calibrated picture saved as calibresult.png")
        print("Calibration Matrix: ")
        print(mtx)
        print("Disortion: ", dist)

        # Save the result
        filename = "./cameraMatrix.txt"
        np.savetxt(filename, mtx, delimiter=',')
        filename = "./cameraDistortion.txt"
        np.savetxt(filename, dist, delimiter=',')

        mean_error = 0
        for i in range(len(obj_points)):
            img_points_2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(img_points[i], img_points_2, cv2.NORM_L2) / len(img_points_2)
            mean_error += error

        print("Mean error: ", mean_error / len(obj_points))
    
    else:
        raise ValueError("In order to calibrate you need at least 9 valid pictures; try again.")
