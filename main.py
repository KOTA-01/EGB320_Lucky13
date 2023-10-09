from vision_module.VisionPiV import Vision
import math

if __name__ == "__main__":
    vision = Vision()  # Create an instance of the Vision class

    while True:
        info = vision.find_information()  # Find and process information
        
        blob_count = info["blob_count"]
        average_distance = info["average_distance"]
        frame_centroid_x = info["frame_centroid_x"]
        frame_centroid_y = info["frame_centroid_y"]

        # Calculate the bearing angle in radians
        bearing_rad = math.atan2(frame_centroid_x, frame_centroid_y)
        bearing_deg = math.degrees(bearing_rad)
        bearing_deg = (bearing_deg + 360) % 360
                
        # print("Asile:", blob_count, "Distance:", average_distance)
        # print("X/Y:", "X_",frame_centroid_x,"Y_", frame_centroid_y)
        # print("Bearing:", bearing_deg)
        
    vision.release_camera()  # Release the camera when done
