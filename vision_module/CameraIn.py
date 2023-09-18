import cv2
class CameraFeedReceiver:
    def process_frame(self, frame):
        # Add your code to process the frame here
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)  # Display the frame and wait for a short time (1 ms)
