from machine import Pin
import time

class Encoder():
    def __init__(self, e1_pin, e2_pin):
        self.encoder_counts = 0
        #Setting the encoder pins to inputs
        self.e1_pin = e1_pin.IN
        self.e2_pin = e2_pin.IN
        #Trigger the rising 
    