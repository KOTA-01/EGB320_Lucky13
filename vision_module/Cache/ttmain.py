import cv2
from vision_module import Vision

class MainVision:
    def __init__(self):
        self.vision = Vision()

    def start_vision(self):
        # Perform any initialization steps before starting vision processing
        self.vision.find_infomation()
        # Start vision processing
        self.vision.find_infomation()

    def stop_vision(self):
        # Perform any cleanup or post-processing steps after stopping vision processing
        self.vision.release_camera()

if __name__ == "__main__":
    main_vision = MainVision()
    
    # Perform any setup or actions before starting vision
    main_vision.setup_something()
    
    main_vision.start_vision()
    
    # Perform any actions after vision processing has started
    main_vision.do_something_else()
    
    main_vision.stop_vision()
    
    # Perform any final cleanup or actions after stopping vision
    main_vision.final_cleanup()
