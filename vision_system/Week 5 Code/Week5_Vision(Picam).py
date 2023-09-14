# import time
# import picamera2
# import numpy as np
# import cv2

# # Simulated data: Distance (in cm) and corresponding pixel counts
# distances = [50, 100, 150, 200, 250]
# pixel_counts = [150, 300, 500, 1000, 2000]



# cap = picamera2.Picamera2()
# config = cap.create_video_configuration(main={"format":'XRGB8888',"size":(640,480)})
# cap.configure(config)
# cap.start()
# while(1):
#     frame = cap.capture_array()
#     frame = cv2.rotate(frame, cv2.ROTATE_180)
#     cv2.imshow("CameraImage", frame)
#     cv2.waitKey(1)
    
# cap.close()
# cap.release()			# Release the camera object (if using opencv)
# cv2.destroyAllWindows()		# Close all opencv pop-up windows