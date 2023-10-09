class WarehouseLayout:
    def __init__(self):
        self.shelf_to_aisle = {
            "0": 0,  # Shelf 0 is in Aisle 0
            "1": 0,  # Shelf 1 is also in Aisle 0
            "2": 1,  # Shelf 2 is in Aisle 1
            "3": 1,  # Shelf 3 is also in Aisle 1
            "4": 2,  # Shelf 4 is in Aisle 2
            "5": 2   # Shelf 4 is also in Aisle 2
        }

        self.aisle_reference_points = {
            0: (164, 100),  # Mid-point for Aisle 0
            1: (99, 100),   # Mid-point for Aisle 1
            2: (33, 100)    # Mid-point for Aisle 2
        }

    def get_aisle_for_shelf(self, shelf_number):
        """Returns the aisle number for a given shelf number."""
        return self.shelf_to_aisle.get(str(shelf_number))

    def get_reference_point_for_aisle(self, aisle_number):
        """Returns the reference point for a given aisle number."""
        return self.aisle_reference_points.get(aisle_number)