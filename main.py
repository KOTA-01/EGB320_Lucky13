from vision_module.Vision import Vision
import math

if __name__ == "__main__":
    vision = Vision()  # Create an instance of the Vision class

    while True:
        info = vision.find_information()  # Find and process information
        
        blob_count = info["blob_count"]
        print(blob_count)
        average_distance = info["average_distance"]
        frame_centroid_x = info["frame_centroid_x"]
        frame_centroid_y = info["frame_centroid_y"]

        # Calculate the bearing angle in radians
        bearing_rad = math.atan2(frame_centroid_x, frame_centroid_y)

        # Convert radians to degrees
        bearing_deg = math.degrees(bearing_rad)

        # Ensure the result is between 0 and 360 degrees
        bearing_deg = (bearing_deg + 360) % 360
                
        
    vision.release_camera()  # Release the camera when done
