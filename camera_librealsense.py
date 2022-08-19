import cv2 as cv
import pyrealsense2 as rs
import time
import numpy as np


class Camera:
    def __init__(self):
        # for librealsense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # for aruco detection
        self.arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        self.arucoParams = cv.aruco.DetectorParameters_create()
        self.corners = None
        self.ids = []

    def start(self):
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        time.sleep(2)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def get_distance(self, frame, add_to_frame=True):
        center, frame = self.get_centers(frame, add_to_frame=True)
        dist = np.linalg.norm(center[0] - center[1], ord=2)
        self.ids = self.ids.flatten()
        i = np.where(self.ids == 22)
        corner = self.corners[i[0][0]].reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corner
        pixel_len = (abs(topLeft[0] - topRight[0]) + abs(bottomLeft[0] - bottomRight[0])) / 2
        length_per_pixel = 0.024 / pixel_len
        real_distance = np.round(dist*length_per_pixel, 3)

        if add_to_frame:
            cv.line(frame, center[0], center[1], (255, 0, 0), 2)
            cv.putText(frame, str(real_distance),
                       (int((center[0][0] + center[1][0]) / 2), center[0][1] - 15), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                       (0, 0, 255), 2)
        return real_distance, frame

    def get_centers(self, frame, add_to_frame=True):
        (self.corners, self.ids, rejected) = cv.aruco.detectMarkers(frame, self.arucoDict, parameters=self.arucoParams)
        centers = []
        for i in range(len(self.corners)):
            corner = self.corners[i].reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corner
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            if add_to_frame:
                # draw the bounding box of the ArUCo detection
                cv.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            if add_to_frame:
                cv.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            centers.append([cX, cY])
        return np.array(centers), frame

    def stop(self):
        self.pipeline.stop()



