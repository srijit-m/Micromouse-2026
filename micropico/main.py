from micropython import const
from micromouse import Micromouse
from maze import Maze
from machine import Pin, Timer
import utime
import time

# maze config
MAZE_WIDTH = const(9)
MAZE_HEIGHT = const(9)
MAZE_GOAL = (4, 4)

# modes
IDLE = const(0)
EXPLORE = const(1)
SPEEDRUN = const(2)

# button times
EXPLORE_DT = const(0)
SPEEDRUN_DT = const(1000)

#Creating micromouse object
mm = Micromouse()

def select_mode():
    mm.led_green_set(0)
    mm.led_blink_red(2)

    # wait for press
    while mm.button.value():
        utime.sleep_ms(5)

    t0 = utime.ticks_ms()
    mode = IDLE

    mm.led_blink_stop()

    while not mm.button.value():
        dt = utime.ticks_diff(utime.ticks_ms(), t0)

        if dt >= SPEEDRUN_DT:
            mm.led_green_set(1)
            mm.led_red_set(0)
            mode = SPEEDRUN
        elif dt >= EXPLORE_DT:
            mm.led_green_set(0)
            mm.led_red_set(1)
            mode = EXPLORE
        else:
            mm.led_green_set(0)
            mm.led_red_set(0)

        utime.sleep_ms(10)

    mm.led_green_set(0)
    mm.led_red_set(0)
    return mode


if __name__ == "__main__":
    maze = Maze(MAZE_WIDTH, MAZE_HEIGHT, MAZE_GOAL)

    mode = select_mode()

    if mode == EXPLORE:
        pass
    elif mode == SPEEDRUN:
        pass

    # main code after here
    """
    while True:
        # mm.move_forward(180)
        # sleep_ms(200)
        for _ in range(8):
            mm.turn(90, 1)
            utime.sleep_ms(250)

        for _ in range(8):
            mm.turn(-90, 1)
            utime.sleep_ms(250)
    """

    # back up
    # mm.move(200)

    # for _ in range(5):
    #     mm.move(180)
    #     utime.sleep_ms(200)

    # mm.move_forward_encoders(1000)

    # # utime.sleep_ms(500)

    # # mm.move_forward(-138)
    # mm.turn(360)

    # mm.move_forward_encoders(100)

    # #### TEST ENCODERS
    # mm.reset_encoders()
    # utime.sleep_ms(1000)
    # mm.led_red_set(1)

    # utime.sleep_ms(15000)

    # e1 = mm.encoder_1_counts()
    # e2 = mm.encoder_2_counts()
    # d1 = mm.encoder_1_distance()
    # d2 = mm.encoder_2_distance()
    # print(f"{e1=}, {e2=}")
    # print(f"{d1=}, {d2=}")
    # ####
    mm.move(180, 1.0)
    time.sleep(1)
    mm.move(180, 1.0)
    time.sleep(1)
    mm.move(180, 1.0)
    time.sleep(1)
    mm.turn(90, 1)
    time.sleep(1)
    mm.move(180, 1.0)
    time.sleep(1)
    mm.turn(90, 1)
    mm.move(180, 1.0)
    time.sleep(1)
    mm.move(180, 1.0)
    time.sleep(1)
    mm.turn(-90, 1.0)


    mm.led_green_set(0)
    mm.led_red_set(1)
