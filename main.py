from vision_module.VisionPiV import Vision

if __name__ == "__main__":
    vision = Vision()  # Create an instance of the Vision class

    while True:
        info = vision.find_information()  # Find and process information
        
        blob_count = info["blob_count"]
        print(blob_count)
        average_distance = info["average_distance"]
        frame_centroid_x = info["frame_centroid_x"]
        frame_centroid_y = info["frame_centroid_y"]
        
        
    vision.release_camera()  # Release the camera when done
