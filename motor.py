"""
Filename: motor.py
Author: Quinn Horton, UQ Mechatronics and Robotics Society
Date: 03/01/2025
Version: 0.5
Description: Provides a software abstraction for the system motors.
License: MIT License
"""
from machine import Pin, PWM
from rotary_irq_rp2 import RotaryIRQ


class Motor():
    """
    Represents the physical GA12-N20 motors in code.

    Parameters:
        m1_pin (int): The pin number used to drive the motor forward.
        m2_pin (int): The pin number used to drive the motor backward.
        e1_pin (int): The pin number used for the forward leading encoder
            signal.
        e1_pin (int): The pin number used for the forward lagging encoder
            signal.
    """

    def __init__(self, m1_pin, m2_pin, e1_pin, e2_pin):
        """
        Initialises the member variables upon first creation.
        """
        self.m1 = PWM(Pin(m1_pin), freq = 2000)
        self.m2 = PWM(Pin(m2_pin), freq = 2000)

        self._encoder = RotaryIRQ(e1_pin, e2_pin)
        self._invert = False
    
    def constrain(self, value, min_value, max_value):
        """
        Constrains a value to remain within a specified range

        Parameters:
            value (int): The value to constrain
            min_value (int): Minimum allowable range of value
            max_value (int): Maximum allowable range of value 
        """
        return min(max_value, max(min_value, value))

    def spin_forward(self, power = 255):
        """
        Turns on the motor to spin max speed in the forward direction.
        Optional power parameter defaulted to max.

        Parameters:
            power (int): Power to drive forwards at. [0, 255]
        """
        limited_power = self.constrain(power, 0, 255)
        self.spin_power(limited_power)

    def spin_backward(self, power = 255):
        """
        Turns on the motor to spin max speed in the reverse direction.
        Optional power parameter defaulted to max.

        Parameters:
            power (int): Power to drive backwards at. [0, 255]
        """
        limited_power = self.constrain(power, 0, 255)
        self.spin_power(limited_power * -1)
        
    def spin_power(self, power):
        """
        Runs the motor to a specified speed with a direction given between -255 and 255.
        Negative power runs the motor backwards.

        Parameters:
            power (int): Desired power to run the motor at. [-255, 255]
        """
        limited_power = self.constrain(power, -255, 255)
        if (limited_power > 0) ^ self._invert:
            self.m2.duty_u16(limited_power * 257)
            self.m1.duty_u16(0)
        else:
            self.m1.duty_u16(limited_power * 257)
            self.m2.duty_u16(0)

    def spin_stop(self):
        """
        Turns off the motor.
        """
        self.m1.duty_u16(0)
        self.m2.duty_u16(0)

    def encoder_read(self):
        """
        Reads the current encoder value.

        Returns:
            int: The rotational frequency of the motor (signed for direction).
        """
        return self._encoder.value()
    
    def invert_motor(self):
        """
        Toggles the default direction for the motor
        """
        self._invert = not self._invert
