import API
import sys
from collections import deque
import time

# WALL STATES
KNOWN_EMPTY = 0
KNOWN_WALL = 1
UNKNOWN_WALL = 2

# DIRECTIONS
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3
DIRECTIONS = (NORTH, EAST, SOUTH, WEST)


# MOVES
FORWARD = 0
TURN_LEFT = 1
TURN_RIGHT = 2
TURN_AROUND = 3


def step(position, direction):
    x, y = position

    if direction == NORTH:
        dx, dy = (0, 1)
    elif direction == EAST:
        dx, dy = (1, 0)
    elif direction == SOUTH:
        dx, dy = (0, -1)
    elif direction == WEST:
        dx, dy = (-1, 0)
    else:
        dx = dy = 0

    return x + dx, y + dy


def left(direction):
    return (direction - 1) % 4


def right(direction):
    return (direction + 1) % 4


def behind(direction):
    return (direction + 2) % 4


class Mouse:

    def __init__(self, start_pos, start_dir):
        self._start_pos = start_pos
        self._start_dir = start_dir
        self._position = start_pos
        self._direction = start_dir

    @property
    def start_position(self):
        return self._start_pos

    @property
    def start_direction(self):
        return self._start_dir

    @property
    def position(self):
        return self._position

    @property
    def direction(self):
        return self._direction

    def reset(self):
        self._position = self._start_pos
        self._direction = self._start_dir

    def turn_left(self):
        self._direction = left(self._direction)

    def turn_right(self):
        self._direction = right(self._direction)

    def turn_around(self):
        self._direction = behind(self._direction)

    def move_forward(self):
        self._position = step(self._position, self._direction)


class Maze:

    def __init__(self, width, height):
        self._width = width
        self._height = height
        self._goal = (width // 2, height // 2)  # assume odd maze size for simplicity

        # initialise empty walls and distances
        self._h_walls = [[UNKNOWN_WALL] * (height + 1) for _ in range(width)]
        self._v_walls = [[UNKNOWN_WALL] * height for _ in range(width + 1)]
        self._dists = [[-1] * height for _ in range(width)]

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def goal(self):
        return self._goal

    @goal.setter
    def goal(self, position):
        self._goal = position

    @property
    def dists(self):
        return self._dists

    @dists.setter
    def dists(self, dists):
        self._dists = dists

    def get_dist(self, position):
        return self._dists[position[0]][position[1]]

    def set_dist(self, position, dist):
        self._dists[position[0]][position[1]] = dist

    def get_wall(self, position, direction):
        """Returns the appropriate wall matrix and the indices for the wall
        adjacent to the specified position

        Returns:
            A tuple (walls, x_idx, y_idx), where the wall is accessed by
            walls[x_idx][y_idx]
        """
        x, y = position
        if direction == NORTH:
            return self._h_walls, x, y + 1
        if direction == EAST:
            return self._v_walls, x + 1, y
        if direction == SOUTH:
            return self._h_walls, x, y
        if direction == WEST:
            return self._v_walls, x, y
        return None

    def within_bounds(self, position):
        """Return whether the position is within the bounds of the maze"""
        x, y = position
        return 0 <= x < self.width and 0 <= y < self.height

    def update_wall(self, position, direction, state):
        """Update the state of a wall if it is unknown and return whether the wall state changed."""
        if not self.within_bounds(position):
            return False

        walls, x, y = self.get_wall(position, direction)

        if walls[x][y] != UNKNOWN_WALL:
            return False

        walls[x][y] = state
        return True

    def is_wall(self, position, direction, assume_wall=False):
        if self.within_bounds(position):
            walls, x, y = self.get_wall(position, direction)
            if walls[x][y] == UNKNOWN_WALL:
                return assume_wall
            return walls[x][y] == KNOWN_WALL
        return False

    def floodfill(self, goal, require_valid_path=False):
        self._dists = [[-1] * self._height for _ in range(self._width)]
        self.set_dist(goal, 0)
        q = deque([goal])
        while q:
            position = q.popleft()
            for direction in DIRECTIONS:
                neighbour = step(position, direction)
                if (
                    self.within_bounds(neighbour)
                    and not self.is_wall(
                        position, direction, assume_wall=require_valid_path
                    )
                    and self.get_dist(neighbour) == -1
                ):
                    self.set_dist(neighbour, self.get_dist(position) + 1)
                    q.append(neighbour)

    def next_direction(self, position, heading, assume_wall=False):
        """Returns the direction to move, based on the current position and

        When multiple neighbouring cells have the same distance from the goal,
        the order of precedence is: forward, left, right, backward.
        """
        directions = (
            heading,
            left(heading),
            right(heading),
            behind(heading),
        )

        x, y = position
        current_dist = self.dists[x][y]

        for direction in directions:
            neighbour = step(position, direction)

            if not self.within_bounds(neighbour) or self.is_wall(
                position, direction, assume_wall=assume_wall
            ):
                continue

            if self.get_dist(neighbour) == current_dist - 1:
                return direction

        return heading


def next_move(direction, target):
    """Returns the required move to make based on the current direction and
    the target direction"""
    offset = (target - direction) % 4

    if offset == 0:
        return FORWARD
    if offset == 1:
        return TURN_RIGHT
    if offset == 2:
        return TURN_AROUND
    return TURN_LEFT  # offset == 3


def update_walls(maze, mouse):
    """Update the known walls in the maze at the mouse's current position.

    Returns:
        Whether any new walls were added.
    """
    position = mouse.position
    direction = mouse.direction
    updated = False

    front_wall_state = KNOWN_WALL if API.wallFront() else KNOWN_EMPTY
    left_wall_state = KNOWN_WALL if API.wallLeft() else KNOWN_EMPTY
    right_wall_state = KNOWN_WALL if API.wallRight() else KNOWN_EMPTY

    updated |= maze.update_wall(position, direction, front_wall_state)
    updated |= maze.update_wall(position, left(direction), left_wall_state)
    updated |= maze.update_wall(position, right(direction), right_wall_state)
    return updated


def extract_path(maze, mouse, require_valid_path=True):
    """Extract the shortest path as a list of directions from start to goal"""
    maze.floodfill(maze.goal, require_valid_path=require_valid_path)

    path = []
    position = mouse.start_position
    direction = mouse.start_direction

    while position != maze.goal:
        next_dir = maze.next_direction(
            position, direction, assume_wall=require_valid_path
        )
        path.append(next_dir)

        position = step(position, next_dir)
        direction = next_dir

    return path


def compress_path(path):
    """Compress repeated directions in a path into (direction, count) tuples"""
    compressed = []
    last_dir = path[0]
    count = 1

    for direction in path[1:]:
        if direction == last_dir:
            count += 1
        else:
            compressed.append((last_dir, count))
            last_dir = direction
            count = 1

    compressed.append((last_dir, count))
    return compressed


def path_to_moves(path, start_dir):
    moves = []
    prev_dir = start_dir
    for direction, count in path:
        move = next_move(prev_dir, direction)
        if move != FORWARD:
            moves.append((move, 1))
        moves.append((FORWARD, count))
        prev_dir = direction
    return moves


def display_walls(maze, mouse):
    """Display the walls at the mouse's current position"""
    for direction in DIRECTIONS:
        if maze.is_wall(mouse.position, direction):
            x, y = mouse.position
            if direction == NORTH:
                API.setWall(x, y, "n")
            elif direction == EAST:
                API.setWall(x, y, "e")
            elif direction == SOUTH:
                API.setWall(x, y, "s")
            elif direction == WEST:
                API.setWall(x, y, "w")


def display_dists(maze):
    for x in range(maze.width):
        for y in range(maze.height):
            dist = maze.get_dist((x, y))
            if dist != -1:
                API.setText(x, y, dist)


def move_forward(mouse):
    mouse.move_forward()
    API.moveForward()


def turn_right(mouse):
    mouse.turn_right()
    API.turnRight()


def turn_around(mouse):
    mouse.turn_around()
    API.turnRight()
    API.turnRight()


def turn_left(mouse):
    mouse.turn_left()
    API.turnLeft()


def turn_to_face(mouse, direction):
    if direction == right(mouse.direction):
        mouse.turn_right()
        API.turnRight()
    elif direction == behind(mouse.direction):
        mouse.turn_around()
        API.turnRight()
        API.turnRight()
    elif direction == left(mouse.direction):
        mouse.turn_left()
        API.turnLeft()


def move_mouse(mouse, move):
    if move == FORWARD:
        move_forward(mouse)
    elif move == TURN_RIGHT:
        turn_right(mouse)
    elif move == TURN_AROUND:
        turn_around(mouse)
    elif move == TURN_LEFT:
        turn_left(mouse)


def search_to(maze, mouse, goal):
    """Move to the target position at a safe speed while mapping the maze."""

    while mouse.position != goal:
        update_walls(maze, mouse)
        maze.floodfill(goal)

        # display walls and distances for debugging
        display_walls(maze, mouse)
        display_dists(maze)

        # determine next move
        target_dir = maze.next_direction(
            mouse.position, mouse.direction, assume_wall=True
        )
        move = next_move(mouse.direction, target_dir)

        # move the mouse
        move_mouse(mouse, move)


def search_maze(maze, mouse):
    """Search to the goal, then search back to the start.

    Assumes that the mouse is at the start position and orientation.
    """
    search_to(maze, mouse, maze.goal)
    search_to(maze, mouse, mouse.start_position)
    turn_to_face(mouse, mouse.start_direction)


def speed_run(moves):
    pass


def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()


WIDTH = API.mazeWidth()
HEIGHT = API.mazeHeight()
START = (0, 0)
HEADING = NORTH

def main():
    maze = Maze(WIDTH, HEIGHT)
    mouse = Mouse(START, HEADING)

    while True:
        search_to(maze, mouse, maze.goal)

        path = extract_path(maze, mouse, require_valid_path=True)
        if extract_path(maze, mouse, require_valid_path=False) == path:
            log("optimal path found")
            break
        else:
            log("non-optimal path found")

        search_to(maze, mouse, mouse.start_position)
        turn_to_face(mouse, mouse.start_direction)

        path = extract_path(maze, mouse, require_valid_path=True)
        if extract_path(maze, mouse, require_valid_path=False) == path:
            log("optimal path found")
            break
        else:
            log("non-optimal path found")

    # fast run
    API.ackReset()
    mouse.reset()

    time.sleep(1)
    log("starting fast run")
    moves = path_to_moves(compress_path(path), HEADING)
    for move, count in moves:
        for _ in range(count):
            API.setColor(*mouse.position, "G")
            move_mouse(mouse, move)


if __name__ == "__main__":
    main()
