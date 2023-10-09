import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow('Thresholder_App')

cv2.createTrackbar("VMax", "Thresholder_App", 0, 255, nothing)
cv2.createTrackbar("VMin", "Thresholder_App", 0, 255, nothing)
cv2.createTrackbar("SMax", "Thresholder_App", 0, 255, nothing)
cv2.createTrackbar("SMin", "Thresholder_App", 0, 255, nothing)
cv2.createTrackbar("HMax", "Thresholder_App", 0, 179, nothing)
cv2.createTrackbar("HMin", "Thresholder_App", 0, 179, nothing)

cap = cv2.VideoCapture(0)

# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 308)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 410)

cv2.setTrackbarPos("VMax", "Thresholder_App", 255)
cv2.setTrackbarPos("VMin", "Thresholder_App", 0)
cv2.setTrackbarPos("SMax", "Thresholder_App", 255)
cv2.setTrackbarPos("SMin", "Thresholder_App", 0)
cv2.setTrackbarPos("HMax", "Thresholder_App", 179)
cv2.setTrackbarPos("HMin", "Thresholder_App", 0)

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    vmax = cv2.getTrackbarPos("VMax", "Thresholder_App")
    vmin = cv2.getTrackbarPos("VMin", "Thresholder_App")
    smax = cv2.getTrackbarPos("SMax", "Thresholder_App")
    smin = cv2.getTrackbarPos("SMin", "Thresholder_App")
    hmax = cv2.getTrackbarPos("HMax", "Thresholder_App")
    hmin = cv2.getTrackbarPos("HMin", "Thresholder_App")

    min_ = np.array([hmin, smin, vmin])
    max_ = np.array([hmax, smax, vmax])

    mask = cv2.inRange(hsv, min_, max_)

    thresholded_img = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Thresholder_App", thresholded_img)

    k = cv2.waitKey(1) & 0xFF

    # exit if 'q' or 'esc' are pressed
    if k == ord('q') or k == 27:
        break

cap.release()
cv2.destroyAllWindows()
