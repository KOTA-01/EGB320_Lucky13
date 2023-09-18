import picamera2
import cv2

# Initialize Pi Camera
camera = picamera2.PiCamera2()
camera.resolution = (640, 480)  # Adjust the resolution as needed

# Create a named window to display the live feed
cv2.namedWindow("Live Camera Feed", cv2.WINDOW_NORMAL)

try:
    for _ in camera.capture_continuous("image.jpg", format="jpeg", use_video_port=True):
        # Read the captured image using OpenCV
        image = cv2.imread("image.jpg")

        # Display the live feed
        cv2.imshow("Live Camera Feed", image)

        # Press 'q' to exit the live feed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Release resources
    camera.close()
    cv2.destroyAllWindows()

