import csv

class OrderReader:
    def __init__(self):
        self.current_order_index = 0

    def ReadOrder(self, filename):
        orders = []

        with open(filename, mode="r", encoding='utf-8-sig') as csv_file:
            csv_reader = csv.reader(csv_file)
            next(csv_reader, None) #skipping header

            for i, row in enumerate(csv_reader):
                if i == self.current_order_index:
                    ItemNumber = row[0]
                    Shelf = row[1]
                    Bay = row[2]
                    Height = row[3]
                    Item = row[4]

                    self.current_order_index +=1

                    return{
                        "item number": ItemNumber,
                        "shelf": Shelf,
                        "height": Height,
                        "bay": Bay,
                        "item": Item
                    }
        print("All orders processed or invalid order index")
        return None
    
    def next_order(self, filename):
        return self.ReadOrder(filename)
        
