from vision_module.Vision import Vision
import cv2 

if __name__ == "__main__":
    vision = Vision()  # Create an instance of the Vision class
    a = 0
    while (a < 10):
        info = vision.find_information()  # Find and process information
        
        blob_count = info["blob_count"]
        print(blob_count)
        average_distance = info["average_distance"]
        frame_centroid_x = info["frame_centroid_x"]
        frame_centroid_y = info["frame_centroid_y"]
        
    

            # Update the state machine and process state based on the information (as shown in previous response)
        cv2.imshow(info["frame"])
        
    vision.release_camera()  # Release the camera when done
