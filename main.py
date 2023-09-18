from vision_module import Vision

if __name__ == "__main__":
    vision = Vision()  # Create an instance of the Vision class

    while True:
        info = vision.find_information()  # Find and process information

        if info is not None:
            # Extract information from the dictionary
            blob_count = info["blob_count"]
            average_distance = info["average_distance"]
            frame_centroid_x = info["frame_centroid_x"]
            frame_centroid_y = info["frame_centroid_y"]

            # Update the state machine and process state based on the information (as shown in previous response)

        # ... (the rest of your main code)

    vision.release_camera()  # Release the camera when done
