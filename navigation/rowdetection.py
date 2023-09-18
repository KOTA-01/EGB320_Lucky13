import numpy as np
import math
import sympy
import cv2
from vision_module.Vision import Vision

class RowMarkerDetector:

    def __init__(self):
        self.currentAisle = -1
        self.rowMarkerPositions = [None, None, None]  # Placeholder, update this to set known row marker positions
        self.vision = Vision()  # Create an instance of the Vision class

    def GetDetectedRowMarker(self):
        rowMarkerRangeBearing = None
        self.currentAisle = -1

        # Get blob info from the Vision class
        blob_info = self.vision.find_information()
        if blob_info is None:
            return None
        
        blob_count = blob_info["blob_count"]

        if blob_count == 1:
            self.currentAisle = 0
        elif blob_count == 2:
            self.currentAisle = 1
        elif blob_count == 3:
            self.currentAisle = 2
        else:
            # no row marker detected
            return None

            # Directly extract the range and bearing (centroid) from the blob_info
        _range = blob_info["average_distance"]  # This gets the average distance from the Vision's find_information
        _bearing = (blob_info["frame_centroid_x"], blob_info["frame_centroid_y"])  # This gets the centroid as a tuple (x, y)
        
        rowMarkerRangeBearing = [_range, _bearing]

        return rowMarkerRangeBearing
    
    def get_detected_row_marker_range(self):
        row_marker_data = self.GetDetectedRowMarker()
        if row_marker_data:
            return row_marker_data[0]  # This gets the range
        return None

    def get_detected_row_marker_bearing(self):
        row_marker_data = self.GetDetectedRowMarker()
        if row_marker_data:
            return row_marker_data[1]  # This gets the bearing
        return None

    def cleanup(self):
        self.vision.release_camera()
