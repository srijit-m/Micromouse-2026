from ucollections import deque
from micropython import const
from grid import *

# WALL STATES
KNOWN_EMPTY = const(0)
KNOWN_WALL = const(1)
UNKNOWN_WALL = const(2)

UNKNOWN_DIST = const(255)
QUEUE_SIZE = const(64)  # I believe 81 is the theoretical max size

SEARCH_SPEED = 1.0
FAST_SPEED = 1.0

class Maze:

    def __init__(self, width, height, goal=None):
        self.width = width
        self.height = height
        self.goal = goal if goal else (width // 2, height // 2)

        # initialise empty walls and distances
        self._h_walls = bytearray([UNKNOWN_WALL for _ in range(width * (height + 1))])
        self._v_walls = bytearray([UNKNOWN_WALL for _ in range((width + 1) * height)])
        self._dists = bytearray([UNKNOWN_DIST for _ in range(width * height)])

    def get_dist(self, position):
        x, y = position
        return self._dists[x + y * self.width]

    def set_dist(self, position, dist):
        x, y = position
        self._dists[x + y * self.width] = dist

    def within_bounds(self, position):
        """Return whether the position is within the bounds of the maze"""
        x, y = position
        return 0 <= x < self.width and 0 <= y < self.height

    def get_wall_array(self, position, direction):
        """Returns the appropriate wall array and the index for the wall
        adjacent to the specified position

        Returns:
            A tuple (wall_array, idx), where the wall is accessed by
            wall_array[idx]
        """
        x, y = position
        if direction == NORTH:
            return self._h_walls, x + (y + 1) * self.width
        if direction == EAST:
            return self._v_walls, (x + 1) + y * (self.width + 1)
        if direction == SOUTH:
            return self._h_walls, x + y * self.width
        if direction == WEST:
            return self._v_walls, x + y * (self.width + 1)
        return None

    def update_wall(self, position, direction, state):
        """Update the state of a wall if it is unknown and return whether the wall state was changed."""
        if not self.within_bounds(position):
            return False

        walls, idx = self.get_wall_array(position, direction)

        if walls[idx] != UNKNOWN_WALL:
            return False

        walls[idx] = state
        return True

    def is_wall(self, position, direction, assume_wall=False):
        if self.within_bounds(position):
            walls, idx = self.get_wall_array(position, direction)
            if walls[idx] == UNKNOWN_WALL:
                return assume_wall
            return walls[idx] == KNOWN_WALL
        return False

    def floodfill(self, goal, require_valid_path=False):
        """Flood the maze, updating the manhattan distances from start to goal.

        When require_valid_path is True, unknown walls are treated as blocked.
        When False, unknown walls are treated as open.
        """
        for i in range(self.width * self.height):
            self._dists[i] = UNKNOWN_DIST
        self.set_dist(goal, 0)

        q = deque((), QUEUE_SIZE)
        q.append(goal)
        while q:
            position = q.popleft()
            for direction in DIRECTIONS:
                neighbour = step(position, direction)
                if (
                    self.get_dist(neighbour) == UNKNOWN_DIST
                    and self.within_bounds(neighbour)
                    and not self.is_wall(
                        position, direction, assume_wall=require_valid_path
                    )
                ):
                    self.set_dist(neighbour, self.get_dist(position) + 1)
                    q.append(neighbour)

    def next_direction(self, position, heading):
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

        current_dist = self.get_dist(position)

        for direction in directions:
            neighbour = step(position, direction)

            if not self.within_bounds(neighbour) or self.is_wall(position, direction):
                continue

            if self.get_dist(neighbour) == current_dist - 1:
                return direction

        return heading


def next_move(direction, target):
    """Returns the required move to make based on the current direction and
    the target direction"""
    offset = (target - direction) & 3

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
    heading = mouse.heading
    updated = False

    front_wall_state = KNOWN_WALL if mouse.wall_front() else KNOWN_EMPTY
    left_wall_state = KNOWN_WALL if mouse.wall_left() else KNOWN_EMPTY
    right_wall_state = KNOWN_WALL if mouse.wall_right() else KNOWN_EMPTY

    updated |= maze.update_wall(position, heading, front_wall_state)
    updated |= maze.update_wall(position, left(heading), left_wall_state)
    updated |= maze.update_wall(position, right(heading), right_wall_state)
    return updated


def extract_path(maze, position, heading, require_valid_path=True):
    """Extract the shortest path as a list of directions from the given position to the goal.

    When require_valid_path is True, unknown walls are treated as blocked.
    When False, unknown walls are treated as open.

    If the paths extracted under both assumptions are identical, then the extracted path is optimal.
    """
    maze.floodfill(maze.goal, require_valid_path)

    path = []

    while position != maze.goal:
        next_dir = maze.next_direction(position, heading)
        path.append(next_dir)

        position = step(position, next_dir)
        heading = next_dir

    return path


def compress_path(path):
    """Encode the direction sequence in a path as (direction, count) tuples."""
    if not path:
        return []

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
    """Convert a compressed direction path into executable move commands."""
    moves = []
    prev_dir = start_dir
    for direction, count in path:
        move = next_move(prev_dir, direction)
        if move != FORWARD:
            moves.append((move, 1))
        moves.append((FORWARD, count))
        prev_dir = direction
    return moves


def extract_moves(maze, position, heading):
    """Return a list of executable moves from the given position to the maze goal
    and whether the resulting path is optimal.
    """
    path_relaxed = extract_path(maze, position, heading, require_valid_path=False)
    path_strict = extract_path(maze, position, heading, require_valid_path=True)

    optimal = path_relaxed == path_strict

    compressed = compress_path(path_strict)
    moves = path_to_moves(compressed, heading)

    return moves, optimal


def move_mouse(mouse, move, speed=1.0):
    if move == FORWARD:
        mouse.move_forward(1, speed=speed)
    elif move == TURN_RIGHT:
        mouse.turn_right(speed=speed)
    elif move == TURN_AROUND:
        mouse.turn_around(speed=speed)
    elif move == TURN_LEFT:
        mouse.turn_left(speed=speed)


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


def speed_run(mouse, moves, speed=FAST_SPEED):
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
