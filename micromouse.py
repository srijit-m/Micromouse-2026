"""
Filename: micromouse.py
Author: Quinn Horton, UQ Mechatronics and Robotics Society
Date: 03/01/2025
Version: 0.5
Description: Provides a software abstraction for the Micromouse robot.
License: MIT License
"""
from machine import Pin, Timer
from motor import Motor


class Micromouse():
    """
    Represents the physical Micromouse device in code.
    Implemented as a singleton class, as the code should only know of the
    device it is running on.
    """
    def __new__(cls):
        """
        Creates instances of the class. Used here to ensure the object is a
        singleton.

        Parameters:
            cls (type): The class itself.

        Returns:
            Micromouse: The single instance of the class.
        """
        if not hasattr(cls, 'instance'):
            cls.instance = super(Micromouse, cls).__new__(cls)
        return cls.instance

    def __init__(self):
        """
        Initialises the member variables upon first creation.
        """
        if hasattr(self, 'exists'):
            return
        self.exists = True

        # Inputs
        self.button = Pin(11, Pin.IN)
        self.ir_1 = Pin(12, Pin.IN)
        self.ir_2 = Pin(13, Pin.IN)
        self.ir_3 = Pin(14, Pin.IN)

        # Outputs
        self.green_led = Pin(10, Pin.OUT)
        self.red_led = Pin(9, Pin.OUT)
        self.debug_led = Pin(25, Pin.OUT)
        self.motor_2 = Motor(17, 18, 15, 16)
        self.motor_1 = Motor(21, 20, 19, 22)
        self.motor_2.invert_motor()

        # Other
        self.blink_timer = Timer()

    def led_set(self, red_val, green_val):
        """
        Set both red and green LEDs to provided values.

        Parameters:
            red_val (bool): The desired state of the red led.
            green_val (bool): The desired state of the green led.
        """
        self.green_led.value(green_val)
        self.red_led.value(red_val)

    def led_green_set(self, value):
        """
        Set the green LED to the provided value.

        Parameters:
            value (bool): The desired state of the green led.
        """
        self.green_led.value(value)

    def led_red_set(self, value):
        """
        Set the red LED to the provided value.

        Parameters:
            value (bool): The desired state of the red led.
        """
        self.red_led.value(value)

    def led_debug_set(self, value):
        """
        Set the debug LED to the provided value.

        Parameters:
            value (bool): The desired state of the debug led.
        """
        self.debug_led.value(value)

    def led_toggle(self):
        """
        Toggle both red and green LEDs when called.
        """
        self.green_led.toggle()
        self.red_led.toggle()

    def led_toggle_start(self, frequency=1):
        """
        Initialise an LED blinking timer for the red and green LEDs.

        Parameters:
            frequency (int, optional): The frequency at which the LEDs should
                blink.
        """
        self.blink_timer.init(mode=Timer.PERIODIC, freq=frequency,
                              callback=lambda t: self.led_toggle())

    def led_toggle_stop(self):
        """
        Stop the blinking of the onboard red and green LEDs and turn them off.
        """
        self.blink_timer.deinit()
        self.red_led.off()
        self.green_led.off()

    def get_ir_values(self, index=0):
        """
        Gets the current values of the infrared object detector sensors.

        Parameters:
            index (int): The number of the IR sensor to read. 1 for IR1
            and 3 for IR3

        Returns:
            Union[(bool, bool, bool), bool]:
                The IR sensor readings in the order 1, 2, 3 if no
                index is provided, otherwise the result of the specified
                sensor. True indicates an object detected.
        """
        if index >= 4:
            raise IndexError("IR Sensor index should not exceed 2.")
        sensor_1 = self.ir_1.value() == 0
        sensor_2 = self.ir_2.value() == 0
        sensor_3 = self.ir_3.value() == 0
        if index == 1:
            return sensor_1
        elif index == 2:
            return sensor_2
        elif index == 3:
            return sensor_3
        elif index < 1:
            return (sensor_1, sensor_2, sensor_3)

    def drive_forward(self, power = 255):
        """
        Turn both motors on to drive forward at full speed.

        Parameters:
            power (int): Optional speed to run motors at
        """
        self.invert_motor_1()
        self.motor_2.spin_forward(power)
        self.motor_1.spin_forward(power)
        #Added this line to get the correct direction

    def drive_backward(self, power = 255):
        """
        Turn both motors on to drive backward at full speed.

        Parameters:
            power (int): Optional speed to run motors at
        """
        self.invert_motor_1()
        self.motor_2.spin_backward(power)
        self.motor_1.spin_backward(power)
    
    def turn_right(self, power = 255):
        self.motor_2.spin_forward(power)
        self.motor_1.spin_forward(power)
    
    def turn_left(self, power = 255):
        self.motor_2.spin_backward(power)
        self.motor_1.spin_backward(power)


    def drive_stop(self):
        """
        Turn off both motors.
        """
        self.motor_2.spin_stop()
        self.motor_1.spin_stop()

    def get_encoders(self):
        """
        Get the rotational frequency of both motor encoders, signed for
            direction.

        Returns:
            (int, int): Encoder 1, and encoder 2 reading respectively
        """
        encoder_2 = self.motor_2.encoder_read()
        encoder_1 = self.motor_1.encoder_read()
        return (encoder_2, encoder_1)
    
    def get_button(self):
        """
        Gets the value of the built-in button
        
        Returns:
            (bool): True if button is pressed
        """
        return self.button.value() < 1
    
    def invert_motor_1(self):
        """
        Toggles the invert direction of motor 1
        """
        self.motor_1.invert_motor()
    
    def invert_motor_2(self):
        """
        Toggles the invert direction of motor 2
        """
        self.motor_2.invert_motor()
