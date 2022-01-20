# **Aruco Marker Tracking**
A Python library for use by the OSU Robotics Club Aerial team to track and estimate the pose of Aruco markers within a camera frame.


# *Getting Started*

## Installation

Install ArucoMarkerTracking from GitHub
```bash
$ pip install git+https://github.com/OSURoboticsClub/Aerial_2021_ArucoMarkerTracking
```

Then, in the python script you wish to use this library in:
```python
from ArucoMarkerTracking import ArucoMultiTracker   #-- for marker tracking functionality
from ArucoMarkerTracking import calibrate_camera   #-- for camera calibration
```
## Camera Calibration
Camera calibration *must* be done before using the ArucoMultiTracker class. You need to have a camera matrix array and a camera distortion array. The `calibrate_camera` function handles the creation of these for you. 

In order to calibrate the camera you must:

1. Print the chessBoard.jpg without any adaption to the page
2. Accurately measure the side of the printed chess board.
3. Mount the chess board on a rigid and flat panel.
4. Call the `calibrate_camera` function. With appropriate parameters for the checkerboard rows, columns, square side length, and camera dimensions. To get good results, take at least 20 different pictures with varying angles and distances.
5. You are done! You can use the output cameraMatrix.txt and cameraDistortion.txt files for the ArucoMultiTracker class.

Example:
```python
from ArucoMarkerTracking import calibrate_camera

# for a 9x6 Checkboard 
calibrate_camera(checkboard_n_rows=9, checkboard_n_cols=6, checkboard_sqr_dim=3.0, 
                 camera_width=1920, camera_height=1080, image_save_path="./calibration/", 
                 image_filename_base="snapshot", image_type="jpg")
```

## ArucoMultiTracker
Tracks Aruco markers from OpenCV camera captures. 

The `ArucoMultiTracker` constructor takes the following arguments:
* ids_to_find
    * This is a list of dicts, where each dict represents the marker to find
    * Format: [{'id': int, 'marker_size': float (in cm)},...]
* camera_matrix
    * The `cameraMatrix.txt` file produced by the `calibrate_camera` function.
* camera_distortion
    * The `cameraDistortion.txt` file produced by the `calibrate_camera` function.
* aruco_dict = aruco.DICT_ARUCO_ORIGINAL
    * The Aruco Marker dictionary to be searching for.
* camera_size
    * The resolution of the camera as list, `[Width, Height]` 
* show_video
    * Whether or not to display the captured camera frame with the detected Aruco marker axis drawn on the frame.

This class is primarily used through the `track()` function:
It takes the following arguments:
* loop -- (bool) determines if the function should loop until the user chooses to quit
* verbose -- (bool) determines verbosity of the function outputs
* show_video -- (bool) determines if the function displays the video being captured with Aruco marker tracking verbosity

The `track()` function returns:
* bool -- Represents if an Aruco marker was detected by this call of the function
* list -- contains dicts that contain pose information for each detected Aruco marker if they are one of the target ids. None if parameter loop == True. The dicitonary keys are keys: {'id', 'x', 'y', 'z','roll', 'pitch', 'yaw'}

Example:
```python
import numpy as np
from ArucoMarkerTracking import AcuroMultiTracker

# ids to find
ids = [{'id': 419, 'marker_size': 10.0}, # marker size is in cm
       {'id': 73, 'marker_size': 18.7}]

calib_path  = "./calibration/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')                                      

tracker = ArucoMultiTracker(ids, camera_matrix, camera_distortion, 
                            camera_size=[1920, 1080], show_video=True)

tracker.track() # shows video capture with detected Aruco markers shown
```


# Troubleshooting

If you encounter a bug, error, or any difficulties using this, please [file an issue](https://github.com/OSURoboticsClub/Aerial_2021_ArucoMarkerTracking/issues).
