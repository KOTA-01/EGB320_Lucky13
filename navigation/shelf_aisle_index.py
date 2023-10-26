class WarehouseLayout:
    def __init__(self):
        self.shelf_to_aisle = {
            "0": 1,  # Shelf 0 is in Aisle 0
            "1": 1,  # Shelf 1 is also in Aisle 0
            "2": 2,  # Shelf 2 is in Aisle 1
            "3": 2,  # Shelf 3 is also in Aisle 1
            "4": 3,  # Shelf 4 is in Aisle 2
            "5": 3   # Shelf 4 is also in Aisle 2
        }

    def get_aisle_for_shelf(self, shelf_number):
        """Returns the aisle number for a given shelf number."""
        return self.shelf_to_aisle.get(str(shelf_number))

    # def get_reference_point_for_aisle(self, aisle_number):
    #     """Returns the reference point for a given aisle number."""
    #     return self.aisle_reference_points.get(aisle_number)