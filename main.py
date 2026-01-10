"""
This file is provided as a sample of basic initialisation and working for
"plug-and-play" of the drivers, but is expected to be altered to implement
system control algorithms.
"""
from micromouse import Micromouse
from machine import Pin

mm = Micromouse()


if __name__ == "__main__":
    while True:
        mm.led_green_set(0)
