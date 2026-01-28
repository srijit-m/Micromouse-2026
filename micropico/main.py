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
MAZE_GOAL = (4, 4)

# modes
IDLE = const(0)
EXPLORE = const(1)
SPEEDRUN = const(2)

# button times in ms
EXPLORE_DT = const(0)
SPEEDRUN_DT = const(800)

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
    """Move to the target position at a safe speed while mapping the maze."""

    while mouse.position != goal:
        # update walls and distances
        if update_walls(maze, mouse):
            maze.floodfill(goal)

        # determine next move
        target_dir = maze.next_direction(mouse.position, mouse.heading)
        move = next_move(mouse.heading, target_dir)

        # move the mouse
        move_mouse(mouse, move, speed=speed)


def search_maze(maze, mouse):
    """Search to the goal, then search back to the start.

    Assumes that the mouse is at the start position and orientation.
    """
    search_to(maze, mouse, maze.goal)
    search_to(maze, mouse, mouse.start_pos)
    mouse.turn_to_face(mouse.start_heading)


def execute_moves(mouse, moves, speed=FAST_SPEED):
    """Execute a precomputed move sequence at speed."""
    for move, count in moves:
        if move == FORWARD:
            mouse.move_forward(count, speed=speed)
        elif move == TURN_RIGHT:
            mouse.turn_right(speed=speed)
        elif move == TURN_AROUND:
            mouse.turn_around(speed=speed)
        elif move == TURN_LEFT:
            mouse.turn_left(speed=speed)


if __name__ == "__main__":
    maze = Maze(MAZE_WIDTH, MAZE_HEIGHT, MAZE_GOAL)

    # TEST ALGORITHM
    maze.update_wall((0, 0), EAST, KNOWN_WALL)
    maze.update_wall((0, 1), EAST, KNOWN_WALL)
    maze.update_wall((0, 2), EAST, KNOWN_WALL)
    maze.update_wall((0, 4), EAST, KNOWN_WALL)
    maze.update_wall((2, 2), EAST, KNOWN_WALL)
    maze.update_wall((2, 2), SOUTH, KNOWN_WALL)
    maze.update_wall((2, 2), WEST, KNOWN_WALL)
    maze.update_wall((2, 3), EAST, KNOWN_WALL)
    maze.update_wall((2, 3), WEST, KNOWN_WALL)
    maze.update_wall((2, 4), WEST, KNOWN_WALL)
    maze.update_wall((4, 0), WEST, KNOWN_WALL)
    maze.update_wall((4, 1), WEST, KNOWN_WALL)
    maze.update_wall((4, 2), NORTH, KNOWN_WALL)

    assert maze.extract_path(
        mm.start_pos, mm.start_heading, require_valid_path=False
    ) == [0, 0, 0, 1, 2, 2, 1, 1, 0, 0, 0, 3, 2, 2]
    # END TEST

    moves = []

    mode = select_mode()

    while True:
        if mode == EXPLORE:
            search_maze(maze, mm)
            moves, optimal = maze.extract_moves(mm.start_pos, mm.start_heading)
            if optimal:
                mm.led_green_set(1)
        elif mode == SPEEDRUN:
            execute_moves(mm, moves)

    # HARDCODED PATH
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
