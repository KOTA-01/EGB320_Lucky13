import time
import picamera
import cv2

def test_camera_presence():
    try:
        with picamera.PiCamera():
            print("Camera detected and initialized.")
    except picamera.PiCameraError:
        print("Camera not detected or unable to initialize.")

def test_image_capture():
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.start_preview()
        time.sleep(2)  # Warm-up time for the camera
        camera.capture('test_image.jpg')
        print("Image captured.")

def test_video_recording():
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.start_preview()
        camera.start_recording('test_video.h264')
        print("Recording video for 5 seconds.")
        time.sleep(5)
        camera.stop_recording()
        print("Video recorded.")

def test_opencv_preview():
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.start_preview()
        time.sleep(10)  # Display preview for 10 seconds

def test_opencv_capture():
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        with picamera.array.PiRGBArray(camera) as output:
            camera.capture(output, 'rgb')
            image = output.array
            cv2.imwrite('test_opencv_capture.jpg', image)
            print("OpenCV capture saved.")

def main():
    print("Raspberry Pi Camera and OpenCV Test")
    test_camera_presence()
    test_image_capture()
    test_video_recording()
    test_opencv_preview()
    test_opencv_capture()
    print("Tests completed.")

if __name__ == "__main__":
    main()
