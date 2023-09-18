import time
import picamera2
import cv2

cap = picamera2.Picamera2()
config = cap.create_video_configuration(main={"format":'XRGB8888',"size":(320,240)})
cap.configure(config)
cap.start()
while(1):
    frame = cap.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    cv2.imshow("CameraImage", frame)
    cv2.waitKey(1)
    
cap.close()
cap.release()			# Release the camera object (if using opencv)
cv2.destroyAllWindows()		# Close all opencv pop-up windows