import time
import cv2
import numpy as np

class Vision:
    def Camera_setup():
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 308)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 410)
        return cap
        
    def PixelDistance_setup():
        distances = [20, 25, 30, 35, 40, 45, 65]
        pixel_width = [178, 146, 119, 102, 86, 78, 54]
        coefficients = np.polyfit(pixel_width, distances, 2)
        pixel_to_distance = np.poly1d(coefficients)
        return pixel_to_distance
        
    def Masking(frame, lower_bound, upper_bound):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
        return mask

if __name__ == '__main__':
    Frequency = 25 #Hz
    Interval = 1.0/Frequency
    # execute control rate loop
    # very simple main robot loop

    Vision.Camera_setup()
    while True:
        print('Executing Loop')
        now = time.time() # get the time
        #do all processing here
        print('a')














        elapsed = time.time() - now # how long was it running?
        time.sleep(Interval-elapsed) # wait for amount of time left from interval
        ## this is not needed but good for debugging rate
        elapsed2 = time.time() - now
        rate2 = 1.0/elapsed2
        print("Processing Rate after sleep is: {}.".format(rate2))
    