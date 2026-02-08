from micropython import const
from micromouse import Micromouse
from maze import Maze, KNOWN_EMPTY, KNOWN_WALL, UNKNOWN_WALL
from grid import *
from machine import Pin, Timer
import utime
import time

# maze config
MAZE_WIDTH = const(9)
MAZE_HEIGHT = const(9)
MAZE_GOAL = (MAZE_WIDTH // 2, MAZE_HEIGHT // 2)

# modes
IDLE = const(0)
EXPLORE = const(1)
SPEEDRUN = const(2)

# button times in ms
EXPLORE_DT = const(0)
SPEEDRUN_DT = const(600)

SEARCH_SPEED = 1.0
FAST_SPEED = 1.0

# Creating micromouse object
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


def update_walls(maze, mouse):
    """Update the known walls in the maze at the mouse's current position.

    Returns:
        Whether any new walls were added.
    """
    position = mouse.position
    heading = mouse.heading
    updated = False

    front_wall_state = KNOWN_WALL if mouse.wall_front() else KNOWN_EMPTY
    left_wall_state = KNOWN_WALL if mouse.wall_left() else KNOWN_EMPTY
    utime.sleep_ms(5)
    right_wall_state = KNOWN_WALL if mouse.wall_right() else KNOWN_EMPTY

    updated |= maze.update_wall(position, heading, front_wall_state)
    updated |= maze.update_wall(position, left(heading), left_wall_state)
    updated |= maze.update_wall(position, right(heading), right_wall_state)
    return updated


def move_mouse(mouse, move, speed=1.0):
    if move == FORWARD:
        mouse.move_cells(1, speed=speed)
    elif move == TURN_RIGHT:
        mouse.turn_right_90(speed=speed)
    elif move == TURN_AROUND:
        mouse.turn_around(speed=speed)
    elif move == TURN_LEFT:
        mouse.turn_left_90(speed=speed)


def search_to(maze, mouse, goal, speed=SEARCH_SPEED):
    """Move to the target position at a safe speed while mapping the maze.

    Return whether the mouse successfully made it to the goal.
    """

    # store these to revert back to in case the mouse crashes
    last_h_walls = maze.get_h_walls()
    last_v_walls = maze.get_v_walls()
    last_dists = maze.get_dists()

    while mouse.position != goal:
        # check if the mouse crashed
        if mouse.has_crashed():
            # revert to old maze state
            maze.set_h_walls(last_h_walls)
            maze.set_v_walls(last_v_walls)
            maze.set_dists(last_dists)

            # reset mouse position and heading
            mouse.reset()

            return False

        # update walls and distances
        update_walls(maze, mouse)
        maze.floodfill(goal)

        # determine next move
        target_dir = maze.next_direction(
            mouse.position, mouse.heading, assume_wall=True
        )
        move = next_move(mouse.heading, target_dir)

        # move the mouse
        move_mouse(mouse, move, speed=speed)

        # TODO: DEBUG add a delay for now
        # try go faster
        # utime.sleep_ms(100)

    return True


def search_maze(maze, mouse):
    """Search to the goal, then search back to the start.

    Assumes that the mouse is at the start position and orientation.
    """
    reached_goal = search_to(maze, mouse, maze.goal)
    if reached_goal:
        # search back to the start
        search_to(maze, mouse, mouse.start_pos)
        mouse.turn_to_face(mouse.start_heading)
        mouse.back_up()
    return reached_goal


def execute_moves(mouse, moves, speed=FAST_SPEED, interval=0):
    """Execute a precomputed move sequence.

    The mouse sleeps for interval milliseconds after each move.
    If the atomic flag is True, forward moves are executed one cell at a time,
    otherwise they are executed as multi-cell motions.
    """
    for move, count in moves:
        if mouse.has_crashed():
            mouse.reset()
            return

        if move == FORWARD:
            mouse.move_cells(count, speed=speed)
        elif move == TURN_RIGHT:
            mouse.turn_right_90(speed=speed)
        elif move == TURN_AROUND:
            mouse.turn_around(speed=speed)
        elif move == TURN_LEFT:
            mouse.turn_left_90(speed=speed)
        utime.sleep_ms(interval)


def calibrate_turns():
    for _ in range(8):
        mm.turn_right_90()
        utime.sleep_ms(250)

    utime.sleep(1)

    for _ in range(8):
        mm.turn_left_90()
        utime.sleep_ms(250)


if __name__ == "__main__":
    maze = Maze(MAZE_WIDTH, MAZE_HEIGHT, MAZE_GOAL)
    moves = []

    while True:
        mode = select_mode()
        utime.sleep_ms(100)

        if mode == EXPLORE:
            # align
            mm.move_to_centre()
            utime.sleep_ms(100)
            if search_maze(maze, mm):
                moves, optimal = maze.extract_moves(mm.start_pos, mm.start_heading)
                if optimal:
                    mm.led_green_set(1)  # green led if optimal path found
        elif mode == SPEEDRUN:
            # align
            mm.move_to_centre()
            utime.sleep_ms(100)

            execute_moves(mm, moves)

        utime.sleep(1)  # so you can see the green led :)
