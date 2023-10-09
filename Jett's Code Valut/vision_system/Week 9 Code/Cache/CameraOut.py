import cv2
class CameraFeedProvider:
    def __init__(self, camera_id=0):
        self.camera = cv2.VideoCapture(camera_id)
        if not self.camera.isOpened():
            raise Exception("Failed to open camera")

    def get_camera_feed(self):
        ret, frame = self.camera.read()
        if not ret:
            raise Exception("Failed to capture frame from camera")
        return frame

    def release_camera(self):
        self.camera.release()
