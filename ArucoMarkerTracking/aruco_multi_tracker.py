""" Based on work from https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/opencv/lib_aruco_pose.py """

import numpy as np
import cv2
import cv2.aruco as aruco
import math, time

class AcuroMultiTracker():
    def __init__(self,
                ids_to_find, # list of dicts [{'id': int, 'marker_size': float (in cm)},...]
                camera_matrix,
                camera_distortion,
                aruco_dict = aruco.DICT_ARUCO_ORIGINAL,
                camera_size=[640,480], # RaspPi
                show_video=False):
        
        self.ids_to_find = sorted(ids_to_find, key=lambda x: x['id'])
        self.show_video = show_video
        
        self.camera_matrix = camera_matrix
        self.camera_distortion = camera_distortion
        
        self.is_detected = False
        self.kill = False
        
        # Create 180 degree rotation matrix about x axis
        self.R_flip = np.zeros((3,3), dtype=np.float32)
        self.R_flip[0,0] = 1.0
        self.R_flip[1,1] =-1.0
        self.R_flip[2,2] =-1.0

        # Define aruco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict)
        self.parameters = aruco.DetectorParameters_create()


        # Capture video camera
        self.capture = cv2.VideoCapture(0)

        # Set the camera size (should be same size as used in calibration)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera_size[0])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])

        # Set cv2 image text font
        self.font = cv2.FONT_HERSHEY_PLAIN

        self.time_read = time.time()
        self.time_detect = self.time_read
        self.fps_read = 0.0
        self.fps_detect = 0.0    


    def rotationMatrixToEulerAngles(self, R):
        """ From a 3x3 rotation matrix R, calculates euler angles. """    
        
        def isRotationMatrix(R):
            """ Returns if the 3x3 rotation matrix, R, is actually a rotation matrix. """

            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6   


        assert (isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])


    def update_fps_read(self):
        """ Updates the fps while in the tracking loop. """
        
        t = time.time()
        self.fps_read = 1.0/(t - self.time_read)
        self.time_read = t
        

    def update_fps_detect(self):
        """ Updates the fps while in the tracking loop while processing detected markers."""
        
        t = time.time()
        self.fps_detect = 1.0 / (t - self.time_detect)
        self.time_detect = t


    def stop(self):
        """ Stop tracking loop. """
        
        self.kill = True


    def get_detected_target_ids(self, ids):
        """ Get the target ids that are detected in the current frame.
        
        Keyword Arguments:
            ids -- the array of ids produced by the cv2.aruco.detectMarkers function

        Returns: 
            bool -- repesents if the target ids are detected in the current frame
            
            list -- contains dicts that represent each target marker that is detected within 
            the current frame 
                Follows the format: [ {'id': int, 'marker_size': float (in cm)}, ...]
        """

        is_none = ids is None
        if not is_none:
            detected_ids = [target_id for target_id in self.ids_to_find if target_id['id'] in ids]
            return True, detected_ids
        else:
            return False, None


    def track(self, loop=True, verbose=False, show_video=None):
        """ Tracks Aruco markers from cv2 camera captures. 
        
        Keyword Arguments:
            loop -- (bool) determines if the function should loop until the user chooses to quit
            
            verbose -- (bool) determines verbosity of the function outputs
            
            show_video -- (bool) determines if the function displays the video being captured with 
            Aruco marker tracking verbosity
        
        Returns:
            bool -- Represents if an Aruco marker was detected by this call of the function
            
            list -- contains dicts that contain pose information for each detected Aruco marker if 
            they are one of the target ids. None if parameter loop == True. Dict keys: {'id', 'x', 'y', 'z','roll', 'pitch', 'yaw'}
        """

        self.kill = False
        
        if show_video is None: 
            show_video = self.show_video
        
        marker_found = False
        outputs = []

        while not self.kill:
            # Read camera frame
            ret, frame = self.capture.read()

            self.update_fps_read()
            
            # Convert image to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Find all the aruco markers in the image
            corners, ids, _ = aruco.detectMarkers(image=gray, 
                            dictionary=self.aruco_dict, 
                            parameters=self.parameters,
                            cameraMatrix=self.camera_matrix, 
                            distCoeff=self.camera_distortion)

            has_detected_target_markers, detected_ids = self.get_detected_target_ids(ids)

            if has_detected_target_markers:
                marker_found = True
                self.update_fps_detect()

                # display fps
                if show_video:
                    cv2.putText(frame, "FPS = %4.0f" % self.fps_detect, (0, 50), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    
                for i, marker in zip(range(len(corners)), detected_ids):
                    # ret = [rvec, tvec, ?]
                    # ret is the array of rotation and position of each marker in camera frame
                    # rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
                    # tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
                    
                    ret = aruco.estimatePoseSingleMarkers(corners[i], 
                                                          marker['marker_size'], 
                                                          self.camera_matrix, 
                                                          self.camera_distortion)
                    
                    # Unpack the output, get only the first
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

                    # Getrotation matrix tag->camera
                    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                    R_tc = R_ct.T
                    
                    # Get the attitude of camera relative to marker
                    roll_camera, pitch_camera, yaw_camera = self.rotationMatrixToEulerAngles(self.R_flip * R_tc)
                    
                    if not loop:
                        outputs.append({'id': marker['id'], 'x': tvec[0], 'y': tvec[1], 'z': tvec[2], 
                                        'roll': math.degrees(roll_camera), 'pitch': math.degrees(pitch_camera), 'yaw': math.degrees(yaw_camera)})
                    
                    # Output marker position and attitute
                    str_position_attitude = ("(%.0f FPS) Marker %d: X = %.1f  Y = %.1f  Z = %.1f, Pitch = %4.0f  Yaw = %4.0f  Roll = %4.0f" 
                                                % (self.fps_detect, marker['id'], tvec[0], tvec[1], tvec[2], 
                                                   math.degrees(roll_camera), math.degrees(pitch_camera), math.degrees(yaw_camera)))
                    
                    # Draw detected marker, put a reference frame over it, and draw detected marker pose information text
                    if show_video:
                        aruco.drawDetectedMarkers(frame, (corners[i],))
                        aruco.drawAxis(frame, self.camera_matrix, self.camera_distortion, rvec, tvec, 1)
                        marker_text = str_position_attitude[str_position_attitude.index('Marker'):]
                        cv2.putText(frame, marker_text, (10, 100 + 50*i), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                   
                    # Print detected marker pose information
                    if verbose: 
                        print(str_position_attitude)
            
            else:
                if verbose: 
                    print("Nothing detected - fps = %.0f" % self.fps_read)
            
            if show_video:
                # Display frame
                cv2.imshow('frame', frame)

                # press 'ESC' to quit
                key = cv2.waitKey(1) & 0xFF
                if key == 27: # ESC
                    self.capture.release()
                    cv2.destroyAllWindows()
                    self.stop()
            
            if not loop:
                return(marker_found, outputs)

        return marker_found, None
