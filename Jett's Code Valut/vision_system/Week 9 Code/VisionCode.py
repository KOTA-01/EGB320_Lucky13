import time 

while 1:
    now = time.time


    elapsed = time.time() - now
    print("Processing Rate after sleep is: {}." .format(1.0/elapsed))