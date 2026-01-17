"""
This file is provided as a sample of basic initialisation and working for
"plug-and-play" of the drivers, but is expected to be altered to implement
system control algorithms.
"""
from micromouse import Micromouse
from machine import Pin
import time

mm = Micromouse()


if __name__ == "__main__":   
    
    mm.led_green_set(1)
    mm.led_red_set(0)
    encoder_tuple  = mm.get_encoders()
    print(encoder_tuple)
    mm.drive_forward(120)
    time.sleep(3)
    mm.drive_stop()
    mm.led_red_set(1)
    mm.led_green_set(0)
    encoder_tuple = mm.get_encoders()
    print(encoder_tuple)
    """
    current_time = time.time()
    while time.time()-current_time < 10:
        mm.led_green_set(1)
        mm.led_red_set(0)
        mm.drive_forward(100)
        encoder_tuple = mm.get_encoders()
        print(encoder_tuple)

    """
    

