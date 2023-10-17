import cv2

# Create a VideoCapture object to access your webcam (0 is usually the default camera)
cap = cv2.VideoCapture(0)

# Loop to continuously capture and display frames
while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Check if the frame was successfully captured
    if not ret:
        print("Error: Could not read frame.")
        break

    # Display the frame in a window
    cv2.imshow("Video", frame)

    # Check for the 'q' key to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the OpenCV window
cap.release()
cv2.destroyAllWindows()
