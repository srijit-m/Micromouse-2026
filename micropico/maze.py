from ucollections import deque
from micropython import const
from grid import *

# WALL STATES
KNOWN_EMPTY = const(0)
KNOWN_WALL = const(1)
UNKNOWN_WALL = const(2)

UNKNOWN_DIST = const(255)
QUEUE_SIZE = const(81)  # 81 should be enough for 9x9 maze


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
        return True  # out of bounds

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
                    self.within_bounds(neighbour)
                    and self.get_dist(neighbour) == UNKNOWN_DIST
                    and not self.is_wall(
                        position, direction, assume_wall=require_valid_path
                    )
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

        current_dist = self.get_dist(position)

        for direction in directions:
            neighbour = step(position, direction)

            if not self.within_bounds(neighbour) or self.is_wall(
                position, direction, assume_wall=assume_wall
            ):
                continue

            if self.get_dist(neighbour) == current_dist - 1:
                return direction

        return None  # unreachable on valid maze TODO: add error handling

    def extract_path(self, position, heading, require_valid_path=True):
        """Extract the shortest path as a list of directions from the given position to the goal.

        When require_valid_path is True, unknown walls are treated as blocked.
        When False, unknown walls are treated as open.

        If the paths extracted under both assumptions are identical, then the extracted path is optimal.
        """
        self.floodfill(self.goal, require_valid_path)

        path = []

        while position != self.goal:
            next_dir = self.next_direction(
                position, heading, assume_wall=require_valid_path
            )
            path.append(next_dir)

            position = step(position, next_dir)
            heading = next_dir

        return path

    @staticmethod
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

    @staticmethod
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

    def extract_moves(self, position, heading):
        """Return a list of executable moves from the given position to the maze goal
        and whether the resulting path is optimal.
        """
        path_relaxed = self.extract_path(position, heading, require_valid_path=False)
        path_strict = self.extract_path(position, heading, require_valid_path=True)

        optimal = path_relaxed == path_strict

        compressed = self.compress_path(path_strict)
        moves = self.path_to_moves(compressed, heading)

        return moves, optimal
