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
        centers, frame = self.get_centers(frame, add_to_frame=True)
        # get idx
        ids = list(self.ids)
        bottom_stat_idx = ids.index(22)
        bottom_dyn_idx = ids.index(32)
        top_stat_idx = ids.index(2)
        top_dyn_idx = ids.index(12)

        dist_bottom = np.linalg.norm(centers[bottom_stat_idx] - centers[bottom_dyn_idx], ord=2)
        # dist_bottom = np.abs(centers[bottom_stat_idx][0] - centers[bottom_dyn_idx][0])
        dist_top = np.linalg.norm(centers[top_stat_idx] - centers[top_dyn_idx], ord=2)
        # dist_top = np.abs(centers[top_stat_idx][0] - centers[top_dyn_idx][0])
        dists = [dist_bottom, dist_top]
        indexs = [bottom_stat_idx, top_stat_idx]
        real_distances = []
        for i in range(len(indexs)):
            corner = self.corners[indexs[i]].reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corner
            pixel_len = (abs(topLeft[0] - topRight[0]) + abs(bottomLeft[0] - bottomRight[0])) / 2
            length_per_pixel = 0.024 / pixel_len
            real_distances.append(np.round(dists[i]*length_per_pixel, 3))

        if add_to_frame:
            ids = [bottom_stat_idx, bottom_dyn_idx, top_stat_idx, top_dyn_idx]
            for i in range(len(ids) // 2):
                cv.line(frame, centers[ids[2*i]], centers[ids[2*i+1]], (255, 0, 0), 2)
                cv.putText(frame, str(real_distances[i]),
                           (int((centers[2*i][0] + centers[2*i+1][0]) / 2), centers[2*i][1] - 15), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                           (0, 0, 255), 2)
        return np.array(real_distances), frame

    def get_centers(self, frame, add_to_frame=True):
        (self.corners, self.ids, rejected) = cv.aruco.detectMarkers(frame, self.arucoDict, parameters=self.arucoParams)
        centers = []
        self.ids = self.ids.flatten()
        # assert len(self.ids) >= 4, "Not all Arucos detected!!!"
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
            # centers[self.ids[i]] = np.array([cX, cY])
        return np.array(centers), frame

    def stop(self):
        self.pipeline.stop()



