import cv2

# Initialize camera
camera = cv2.VideoCapture(0)  # Use appropriate camera index if not 0

# Test camera compatibility
while True:
    ret, frame = camera.read()
    if not ret:
        print("Error capturing frame")
        break
    
    # Display camera feed
    cv2.imshow("Camera Feed", frame)
    
    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release camera
camera.release()
cv2.destroyAllWindows()
