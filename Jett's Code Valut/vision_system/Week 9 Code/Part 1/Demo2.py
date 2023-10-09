import time
if __name__ == '__main__':
    Frequency = 25 #Hz
    Interval = 1.0/Frequency
    # execute control rate loop
    # very simple main robot loop
    while True:
        print('Executing Loop')
        now = time.time() # get the time
        #do all processing here

        















        elapsed = time.time() - now # how long was it running?
        time.sleep(Interval-elapsed) # wait for amount of time left from interval
        ## this is not needed but good for debugging rate
        elapsed2 = time.time() - now
        rate2 = 1.0/elapsed2
        print("Processing Rate after sleep is: {}.".format(rate2))
    