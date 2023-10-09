from CameraIn import CameraFeedReceiver
from CameraOut import CameraFeedProvider
import cv2
if __name__ == "__main__":
    camera_provider = CameraFeedProvider()  # Initialize the camera feed provider
    camera_receiver = CameraFeedReceiver()  # Initialize the camera feed receiver

    try:
        while True:
            frame = camera_provider.get_camera_feed()  # Capture a frame from the camera
            camera_receiver.process_frame(frame)  # Process the frame in the second class

    except KeyboardInterrupt:
        pass  # Handle keyboard interrupt (e.g., Ctrl+C)

    finally:
        camera_provider.release_camera()  # Release the camera when done
        cv2.destroyAllWindows()  # Close any open windows
