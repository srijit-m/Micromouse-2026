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
    mm.move_forward(20, 120)
    mm.led_red_set(0)
    mm.led_green_set(1)
    
    

