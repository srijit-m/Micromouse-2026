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
    
    
    mm.move_one_cell()
    time.sleep(1)
    mm.move_one_cell()
    time.sleep(1)
    mm.move_one_cell()
    time.sleep(1)
    mm.turn(90, 1)
    time.sleep(1)
    mm.move_one_cell()
    time.sleep(1)
    mm.turn(90, 1)
    mm.move_one_cell()
    time.sleep(1)
    mm.move_one_cell()
    time.sleep(1)
    mm.turn(-90, 1.0)
    mm.move_one_cell()
    time.sleep(1)
    mm.move_one_cell()
    mm.turn(-90, 1.0)
    mm.move_one_cell()
    time.sleep(1)
    mm.move_one_cell()
    time.sleep(1)
    mm.move_one_cell()
    time.sleep(1)
    mm.turn(-90, 1.0)
    mm.move_one_cell()
    time.sleep(1)
    mm.turn(-90, 1.0)
    time.sleep(1)
    mm.move_one_cell()
    mm.move_one_cell()
    mm.led_green_set(1)
    mm.led_red_set(0)
