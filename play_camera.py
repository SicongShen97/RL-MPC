from camera_librealsense import Camera
import cv2 as cv
camera = Camera()
camera.start()

while True:
    frame = camera.get_frame()
    distance, frame = camera.get_distance(frame, add_to_frame=True)
    # centers, frame = camera.get_centers(frame, add_to_frame=True)
    # print(centers)
    cv.imshow('frame', frame)
    key = cv.waitKey(1)
    if key == ord('q'):
        break

cv.destroyAllWindows()
camera.stop()

