import API
import sys
from collections import deque
from enum import Enum

Position = tuple[int, int]


# (row, col) coordinate system
class Direction(Enum):
    # direction deltas (dr, dc)
    NORTH = (-1, 0)
    EAST = (0, 1)
    SOUTH = (1, 0)
    WEST = (0, -1)

    @property
    def dr(self) -> int:
        return self.value[0]

    @property
    def dc(self) -> int:
        return self.value[1]

    def rotate(self, steps):
        directions = list(type(self))
        i = directions.index(self)
        return directions[(i + steps) % len(directions)]


class Move(Enum):
    FORWARD = 0
    RIGHT = 1
    BACKWARD = 2
    LEFT = 3
    NONE = 4


class Maze:
    def __init__(self, width, height) -> None:
        self._height = height
        self._width = width
        self._goal = (height // 2, width // 2)  # assume odd maze size for simplicity
        self._h_walls = [[False] * width for _ in range(height + 1)]
        self._v_walls = [[False] * (width + 1) for _ in range(height)]
        self._dists = [[-1] * width for _ in range(height)]

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def goal(self) -> Position:
        return self._goal

    @goal.setter
    def goal(self, pos: Position) -> None:
        self._goal = pos

    @property
    def dists(self) -> list[list[int]]:
        return self._dists

    def get_wall(
        self, row, col, direction: Direction
    ) -> tuple[list[list[bool]], int, int]:
        """Returns the matrix representing horizontal or vertical walls and the
        indexes corresponding to the wall at the specified position"""
        if direction == Direction.NORTH:
            return self._h_walls, row, col
        if direction == Direction.EAST:
            return self._v_walls, row, col + 1
        if direction == Direction.SOUTH:
            return self._h_walls, row + 1, col
        if direction == Direction.WEST:
            return self._v_walls, row, col

    def add_wall(self, row, col, direction) -> None:
        walls, r, c = self.get_wall(row, col, direction)
        walls[r][c] = True

    def is_wall(self, row, col, direction: Direction) -> bool:
        walls, r, c = self.get_wall(row, col, direction)
        return walls[r][c]

    def neighbours(self, row, col) -> list[tuple[int, int, Direction]]:
        """Returns the accessible neighbouring cells from the specified position,
        i.e. cells that are within bounds and not blocked by a wall.

        Returns:
            List of tuples where each tuple contains (row, col, direction)
        """
        neighbours = []
        for d in Direction:
            nr, nc = row + d.dr, col + d.dc
            if 0 <= nr < self._height and 0 <= nc < self._width:  # within bounds
                if not self.is_wall(row, col, d):  # not blocked by a wall
                    neighbours.append((nr, nc, d))
        return neighbours

    def floodfill(self) -> None:
        self._dists = [[-1] * self._width for _ in range(self._height)]
        self._dists[self._goal[0]][self._goal[1]] = 0
        q = deque([self._goal])
        while q:
            row, col = q.popleft()
            for nr, nc, _ in self.neighbours(row, col):
                if self._dists[nr][nc] == -1:  # check cell is blank
                    self._dists[nr][nc] = self._dists[row][col] + 1
                    q.append((nr, nc))

    def next_direction(self, row, col, current_dir: Direction) -> Direction:
        """Updates the manhattan distances using floodfill, then returns the
        direction to move, based on the current position"""
        self.floodfill()
        dists = self.dists
        min_dist = dists[row][col]
        next_dir = current_dir
        # find neighbouring cell with lowest distance from goal
        for nr, nc, direction in self.neighbours(row, col):
            dist = dists[nr][nc]
            if dist < min_dist:
                min_dist = dist
                next_dir = direction
        return next_dir

    def next_move(self, current_dir: Direction, next_dir: Direction) -> Move:
        """Returns the required move to make based on the current direction and
        the next direction"""
        # take advantage of the fact that the Direction enum is ordered clockwise
        dirs = list(Direction)
        offset = (dirs.index(next_dir) - dirs.index(current_dir)) % len(dirs)

        if offset == 0:
            return Move.FORWARD
        if offset == 1:
            return Move.RIGHT
        if offset == 2:
            return Move.BACKWARD
        if offset == 3:
            return Move.LEFT
        return Move.NONE


def rc_to_xy(row, col, num_rows):
    """Convert from (row, col) to (x, y) coordinates"""
    return col, num_rows - 1 - row


def display_walls(maze: Maze, row, col):
    for direction in Direction:
        if maze.is_wall(row, col, direction):
            x, y = rc_to_xy(row, col, maze.height)
            if direction == Direction.NORTH:
                API.setWall(x, y, "n")
            elif direction == Direction.EAST:
                API.setWall(x, y, "e")
            elif direction == Direction.SOUTH:
                API.setWall(x, y, "s")
            elif direction == Direction.WEST:
                API.setWall(x, y, "w")


def display_dists(maze: Maze):
    dists = maze.dists
    for row_idx, row in enumerate(dists):
        for col_idx, dist in enumerate(row):
            API.setText(*rc_to_xy(row_idx, col_idx, maze.height), dist)


def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()


def update_walls(maze: Maze, row, col, direction):
    if API.wallFront():
        maze.add_wall(row, col, direction)
    if API.wallLeft():
        maze.add_wall(row, col, direction.rotate(-1))
    if API.wallRight():
        maze.add_wall(row, col, direction.rotate(1))


def move_mouse(maze, row, col, direction):
    """Moves the mouse and returns the new position and direction of movement"""
    next_dir = maze.next_direction(row, col, direction)
    move = maze.next_move(direction, next_dir)
    if move == Move.FORWARD:
        API.moveForward()
    elif move == Move.RIGHT:
        API.turnRight()
        API.moveForward()
    elif move == Move.BACKWARD:
        API.turnRight()
        API.turnRight()
        API.moveForward()
    elif move == Move.LEFT:
        API.turnLeft()
        API.moveForward()
    return row + next_dir.dr, col + next_dir.dc, next_dir


def main():
    # the API uses (x, y) cartesian coordinates
    # the Maze class uses (r, c) coordinates
    width = API.mazeWidth()
    height = API.mazeHeight()
    assert width and height

    # starting position and direction
    start = (height - 1, 0)
    row, col = start
    direction = Direction.NORTH

    maze = Maze(width, height)

    while True:
        # mouse goes back and forth between start and centre
        if maze.goal == (row, col):
            if maze.goal == start:
                maze.goal = height // 2, width // 2
            else:
                log("Centre reached")
                maze.goal = start

        # update walls and recalculate distances
        update_walls(maze, row, col, direction)
        maze.floodfill()

        # display walls and distances for debugging
        display_walls(maze, row, col)
        display_dists(maze)

        # move the mouse
        row, col, direction = move_mouse(maze, row, col, direction)


if __name__ == "__main__":
    main()
