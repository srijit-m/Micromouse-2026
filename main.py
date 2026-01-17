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
    
    mm.led_red_set(1)
    mm.led_green_set(0)
    encoder_1_reading = mm.get_encoder_1_counts()
    print(encoder_1_reading)
    mm.drive_forward(100)
    while (mm.get_encoder_1_counts() < 1000):
        print(mm.get_encoder_1_counts())
        time.sleep_ms(10)

    mm.drive_stop()
    mm.led_red_set(0)
    mm.led_green_set(1)
    
    """
    current_time = time.time()
    while time.time()-current_time < 10:
        mm.led_green_set(1)
        mm.led_red_set(0)
        mm.drive_forward(100)
        encoder_tuple = mm.get_encoders()
        print(encoder_tuple)

    """
    

