from micropython import const
from machine import Pin, Timer, I2C
from motor import Motor
from encoder_portable import Encoder
from pid import PID
from grid import *
import math
import utime
from vl6180x import VL6180X

START_POS = (0, 0)
START_HEADING = NORTH

CELL_SIZE_MM = 180

WHEEL_DIAMETER = const(44)  # mm
ENCODER_1_COUNTS_PER_REV = const(4280)
ENCODER_2_COUNTS_PER_REV = const(4280)
ENCODER_DIFF_PER_REV = const(17825)  # drifts about 3mm forward each 360 degrees turn

MM_PER_REV = 3.14159 * WHEEL_DIAMETER

# compensate for the mouse undershooting left turns
LEFT_TURN_CORRECTION = 1.004

PID_DT = 0.010  # seconds

# tested with dt = 0.01
KP_DIST = 2.7
KD_DIST = 0.25
KP_ANGLE = 0.5
KD_ANGLE = 0.03

# in counts
DIST_THRESHOLD = const(75)  # 2.5mm
ANGLE_THRESHOLD = const(100)  # 2 degrees

# min pwm that motors can move at (actually 90 but didn't work well with pid)
MIN_PWM = const(150)
MAX_PWM = const(255)

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

        self.start_pos = START_POS
        self.start_heading = START_HEADING

        self.position = START_POS
        self.heading = START_HEADING

        # Inputs
        self.button = Pin(11, Pin.IN, Pin.PULL_UP)
        self.ir_1 = Pin(12, Pin.IN)
        self.ir_2 = Pin(13, Pin.IN)
        self.ir_3 = Pin(14, Pin.IN)

        # Outputs
        self.green_led = Pin(10, Pin.OUT)
        self.red_led = Pin(9, Pin.OUT)
        self.debug_led = Pin(25, Pin.OUT)
        self.motor_1 = Motor(21, 20)
        self.motor_2 = Motor(17, 18)

        self.encoder_1 = Encoder(22, 19)
        self.encoder_2 = Encoder(8, 15)

        self.controller = Controller()

        # Other
        self.blink_timer = Timer()

        i2c = I2C(1, scl=Pin(3), sda=Pin(2))

        self.xshut_left = Pin(7, Pin.OUT)
        self.xshut_right = Pin(6, Pin.OUT)

        # Hold both in reset
        self.xshut_left.value(0)
        self.xshut_right.value(0)
        utime.sleep_ms(500)

        # -------------------------
        # LEFT SENSOR
        # -------------------------
        # LEFT SENSOR
        self.xshut_left.value(0)
        utime.sleep_ms(200)

        self.xshut_left.value(1)
        utime.sleep_ms(500)

        # Wait until it appears on the bus
        for _ in range(20):
            if 0x29 in i2c.scan():
                break
            utime.sleep_ms(10)
        else:
            raise RuntimeError("Left VL6180X not detected")

        self.left_sensor = VL6180X(i2c, 0x29, 5)
        self.left_sensor.set_address(0x30)
        utime.sleep_ms(500)

        # -------------------------
        # RIGHT SENSOR
        # -------------------------
        self.xshut_right.value(0)
        utime.sleep_ms(500)

        self.xshut_right.value(1)
        utime.sleep_ms(500)

        for _ in range(20):
            if 0x29 in i2c.scan():
                break
            utime.sleep_ms(10)
        else:
            raise RuntimeError("Right VL6180X not detected")
        

        self.right_sensor = VL6180X(i2c, 0x29, 5)
        self.right_sensor.set_address(0x31)
        utime.sleep_ms(500)

        # -------------------------
        # FRONT SENSOR (separate bus, no XSHUT)
        # -------------------------
        self.front_sensor = VL6180X(
            I2C(0, scl=Pin(1), sda=Pin(0)),
            0x29,
            5
        )

    
    def get_position(self):
        return self.position

    def get_heading(self):
        return self.heading

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

    def led_toggle_red(self):
        self.red_led.toggle()

    def led_toggle_green(self):
        self.green_led.toggle()

    def led_blink_start(self, frequency=1):
        """
        Initialise an LED blinking timer for the red and green LEDs.

        Parameters:
            frequency (int, optional): The frequency at which the LEDs should
                blink.
        """
        self.blink_timer.deinit()
        self.blink_timer.init(mode=Timer.PERIODIC, freq=frequency,
                              callback=lambda t: self.led_toggle())

    def led_blink_stop(self):
        """
        Stop the blinking of the onboard red and green LEDs and turn them off.
        """
        self.blink_timer.deinit()
        self.red_led.off()
        self.green_led.off()

    def led_blink_red(self, frequency=1):
        self.blink_timer.deinit()
        self.blink_timer.init(
            mode=Timer.PERIODIC,
            freq=frequency,
            callback=lambda t: self.led_toggle_red(),
        )

    def led_blink_green(self, frequency=1):
        self.blink_timer.deinit()
        self.blink_timer.init(
            mode=Timer.PERIODIC,
            freq=frequency,
            callback=lambda t: self.led_toggle_green(),
        )

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

    def wall_front(self):
        return False

    def wall_left(self):
        return False

    def wall_right(self):
        return False

    def drive(self, power=255):
        self.motor_1.spin_power(power)
        self.motor_2.spin_power(power)

    def drive_stop(self):
        """
        Turn off both motors.
        """
        self.motor_2.spin_stop()
        self.motor_1.spin_stop()

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

    def reset_encoders(self):
        self.encoder_1.reset()
        self.encoder_2.reset()

    def encoder_1_counts(self):
        return self.encoder_1.read()

    def encoder_2_counts(self):
        return self.encoder_2.read()

    def avg_encoder_counts(self):
        return (self.encoder_1_counts() + self.encoder_2_counts()) // 2

    def encoder_1_distance(self):
        """Return linear distance (mm) travelled by motor 1 as measured by encoder"""
        revolutions = self.encoder_1_counts() / ENCODER_1_COUNTS_PER_REV
        return revolutions * MM_PER_REV

    def encoder_2_distance(self):
        """Return linear distance (mm) travelled by motor 2 as measured by encoder"""
        revolutions = self.encoder_2_counts() / ENCODER_2_COUNTS_PER_REV
        return revolutions * MM_PER_REV

    def move(self, distance, speed=1.0):
        """Move forward by the specified distance in mm. If the distance is negative,
        move backward instead.
        """
        self.reset_encoders()
        self.controller.reset()

        self.controller.set_goal_distance(distance)
        self.controller.set_goal_angle(0)

        self.update_motors(speed)

    def turn(self, angle, speed=1.0):
        """Turn right by the specified angle in degrees. If the angle is negative,
        turn left instead.
        """
        self.reset_encoders()
        self.controller.reset()

        self.controller.set_goal_distance(0)
        self.controller.set_goal_angle(angle)

        self.update_motors(speed)

    def update_motors(self, speed=1.0):
        """Run PID control loop on motors until the mouse is at the goal"""
        last = utime.ticks_us()
        while True:
            now = utime.ticks_us()
            dt = (utime.ticks_diff(now, last)) / 1_000_000  # seconds
            if dt >= PID_DT:
                last = now

                enc1 = self.encoder_1_counts()
                enc2 = self.encoder_2_counts()

                # print(f"{enc1=}, {enc2=}")

                pwm_1, pwm_2 = self.controller.update(enc1, enc2, dt)

                self.motor_1.spin_power(int(pwm_1 * speed))
                self.motor_2.spin_power(int(pwm_2 * speed))

            if self.controller.at_goal():
                break

        self.drive_stop()
    
    def reset_tof(self, pin):
        pin.value(0)
        utime.sleep_ms(20)
        pin.value(1)
        utime.sleep_ms(20)
    
    def read_tof_sensors(self):
        """Returns time of flight sensor readings as a tuple in the order front, left, right"""
        front_sensor_val = self.front_sensor._read_range_single()
        utime.sleep_ms(50)
        left_sensor_val = self.left_sensor._read_range_single()
        utime.sleep_ms(50)
        right_sensor_val = self.right_sensor._read_range_single()
        utime.sleep_ms(50)
        return front_sensor_val, left_sensor_val, right_sensor_val
    
    def wall_align_side(self):
        """This function is for aligning with the left or right walls"""
        #First get sensor readings
        _, left_distance, right_distance = self.read_tof_sensors()
        #Check for a wall to the right
        if right_distance < 75:
            #There is a wall to the right
            if right_distance > 57:
                self.turn(5, 1.0)               
            elif right_distance < 47:
               self.turn(-5, 1.0)
        elif left_distance < 75: 
            if left_distance > 54:
                self.turn(-5, 1.0)
            elif left_distance < 44:
                self.turn(5, 1.0)
    
    def wall_align_front(self):
        """This function is for aligning with the front wall"""
        front_distance, _, _ = self.read_tof_sensors()
        if (front_distance < 80):
            #There is a wall in front
            error = front_distance - 60
            self.move(error, 1.0)

    
    def move_one_cell(self):
        self.move(180, 1.0)
        utime.sleep_ms(100)
        self.wall_align_side()
        utime.sleep_ms(50)
        self.wall_align_front()
        utime.sleep_ms(50)

    def move_cells(self, n=1, speed=1.0):
        """Move forward/backward n cells and update the internal position state"""
        self.move(n * CELL_SIZE_MM, speed)
        self.position = step(self.position, self.heading, n)

    def turn_right_90(self, speed=1.0):
        """Turn right 90 degrees and update the internal heading states"""
        self.turn(90, speed)
        self.heading = right(self.heading)

    def turn_left_90(self, speed=1.0):
        """Turn left 90 degrees and update the internal heading state"""
        self.turn(-90, speed)
        self.heading = left(self.heading)

    def turn_around(self, speed=1.0):
        """Turn 180 degrees and update the internal heading state"""
        self.turn(180, speed)
        self.heading = behind(self.heading)

    def turn_to_face(self, direction, speed=1.0):
        """Turn to face the specified direction and update the internal heading state"""
        delta = (direction - self.heading) * 90
        delta = (delta + 180) % 360 - 180
        self.turn(delta, speed)
        self.heading = direction

    def back_up(self, speed=MIN_PWM, timeout=1000):
        """Drive the mouse backwards to align with a back wall.
        Uses the encoders to tell when wall has been reached."""
        self.drive(-abs(speed))

        start_time = utime.ticks_ms()
        prev_counts = self.avg_encoder_counts()
        while utime.ticks_diff(utime.ticks_ms(), start_time) < timeout:
            utime.sleep_ms(50)
            counts = self.avg_encoder_counts()
            if counts == prev_counts:
                break
            prev_counts = counts
        self.drive_stop()

    def move_forward_encoders(self, distance):
        self.reset_encoders()
        revs = distance / (math.pi * WHEEL_DIAMETER)
        required_counts = revs * ENCODER_1_COUNTS_PER_REV
        rounded_counts = int(required_counts)
        self.drive(255)
        while abs(self.encoder_1_counts()) < rounded_counts:
            # print(f"{self.encoder_1_counts()}")
            pass
        self.drive_stop()

    def turn_left_encoders(self, power):
        """A 90 degree left turn using encoders"""
        self.reset_encoders()
        difference = 0
        goal_difference = 985
        self.turn_left_90(power)
        while (difference < goal_difference):
            difference = self.encoder_1_counts() - self.encoder_2_counts()
        self.drive_stop()


class Controller:

    def __init__(self):
        self._distance_controller = PID(KP_DIST, KD_DIST)
        self._angle_controller = PID(KP_ANGLE, KD_ANGLE)

        self._goal_counts = 0
        self._goal_difference = 0

        self._at_goal = False

    def set_goal_distance(self, distance_mm):
        revolutions = distance_mm / MM_PER_REV
        self._goal_counts = int(revolutions * ENCODER_1_COUNTS_PER_REV)

    def set_goal_angle(self, angle_degrees):
        if angle_degrees < 0:
            angle_degrees *= LEFT_TURN_CORRECTION
        self._goal_difference = int(angle_degrees / 360 * ENCODER_DIFF_PER_REV)

    def at_goal(self):
        return self._at_goal

    def apply_deadband(self, value, min_pwm=MIN_PWM):
        if value > 0:
            return max(value, min_pwm)
        elif value < 0:
            return min(value, -min_pwm)
        return 0

    def update(self, encoder_1, encoder_2, dt):
        dist_error = self._goal_counts - (encoder_1 + encoder_2) / 2
        angle_error = self._goal_difference - (encoder_2 - encoder_1)

        self._at_goal = (
            abs(dist_error) < DIST_THRESHOLD and abs(angle_error) < ANGLE_THRESHOLD
        )

        forward = self._distance_controller.update(dist_error, dt)
        turn = self._angle_controller.update(angle_error, dt)

        left = int(forward - turn)
        right = int(forward + turn)

        left = self.apply_deadband(left)
        right = self.apply_deadband(right)

        left = max(-MAX_PWM, min(MAX_PWM, left))
        right = max(-MAX_PWM, min(MAX_PWM, right))

        return left, right

    def reset(self):
        self._distance_controller.reset()
        self._angle_controller.reset()
        self._goal_counts = 0
        self._goal_difference = 0
        self._at_goal = False
