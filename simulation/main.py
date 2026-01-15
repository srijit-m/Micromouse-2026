import API
import sys
from collections import deque
from enum import Enum


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

    def rotate(self, steps: int):
        directions = list(type(self))
        i = directions.index(self)
        return directions[(i + steps) % len(directions)]


class Move(Enum):
    FORWARD = 0
    RIGHT = 1
    BACKWARD = 2
    LEFT = 3


class Maze:
    def __init__(self, width: int, height: int) -> None:
        self._height = height
        self._width = width
        self._goal = (height // 2, width // 2)  # assume odd maze size for simplicity

        # starting position and direction
        self._mouse_pos: tuple[int, int] = (height - 1, 0)
        self._mouse_dir: Direction = Direction.NORTH

        # initialise empty walls and distances
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
    def goal(self) -> tuple[int, int]:
        return self._goal

    @goal.setter
    def goal(self, position: tuple[int, int]) -> None:
        self._goal = position

    @property
    def mouse_position(self) -> tuple[int, int]:
        return self._mouse_pos

    @property
    def mouse_direction(self) -> Direction:
        return self._mouse_dir

    @property
    def dists(self) -> list[list[int]]:
        return self._dists

    def get_wall(
        self, position: tuple[int, int], direction: Direction
    ) -> tuple[list[list[bool]], int, int]:
        """Returns the matrix representing horizontal or vertical walls and the
        indexes corresponding to the wall at the specified position"""
        row, col = position
        if direction == Direction.NORTH:
            return self._h_walls, row, col
        if direction == Direction.EAST:
            return self._v_walls, row, col + 1
        if direction == Direction.SOUTH:
            return self._h_walls, row + 1, col
        if direction == Direction.WEST:
            return self._v_walls, row, col
        return None

    def add_wall(self, position: tuple[int, int], direction: Direction) -> None:
        if 0 <= position[0] < self.height and 0 <= position[1] < self.width:
            walls, r, c = self.get_wall(position, direction)
            walls[r][c] = True

    def is_wall(self, position: tuple[int, int], direction: Direction) -> bool:
        if 0 <= position[0] < self.height and 0 <= position[1] < self.width:
            walls, r, c = self.get_wall(position, direction)
            return walls[r][c]
        return False

    def neighbours(self, position: tuple[int, int]) -> list[tuple[int, int, Direction]]:
        """Returns the accessible neighbouring cells from the specified position,
        i.e. cells that are within bounds and not blocked by a wall.

        Returns:
            List of tuples where each tuple contains (row, col, direction)
        """
        row, col = position
        neighbours = []
        for d in Direction:
            nr, nc = row + d.dr, col + d.dc
            if 0 <= nr < self._height and 0 <= nc < self._width:  # within bounds
                if not self.is_wall(position, d):  # not blocked by a wall
                    neighbours.append((nr, nc, d))
        return neighbours

    def floodfill(self) -> None:
        self._dists = [[-1] * self._width for _ in range(self._height)]
        self._dists[self._goal[0]][self._goal[1]] = 0
        q = deque([self._goal])
        while q:
            position = q.popleft()
            for nr, nc, _ in self.neighbours(position):
                if self._dists[nr][nc] == -1:  # check cell is blank
                    self._dists[nr][nc] = self._dists[position[0]][position[1]] + 1
                    q.append((nr, nc))

    def next_direction(self) -> Direction:
        """Returns the direction to move, based on the current position.

        Assumes that distances have been calculated already
        """
        dists = self.dists
        position = self._mouse_pos
        min_dist = dists[position[0]][position[1]]
        next_dir = self._mouse_dir
        # find neighbouring cell with lowest distance from goal
        for nr, nc, direction in self.neighbours(position):
            dist = dists[nr][nc]
            if dist != -1 and dist < min_dist:
                min_dist = dist
                next_dir = direction
        return next_dir

    def next_move(self, next_dir: Direction) -> Move:
        """Returns the required move to make based on the current direction and
        the next direction"""
        # for simplicity, take advantage of the fact that the Direction enum is ordered clockwise
        dirs = list(Direction)
        offset = (dirs.index(next_dir) - dirs.index(self._mouse_dir)) % len(dirs)

        if offset == 0:
            return Move.FORWARD
        if offset == 1:
            return Move.RIGHT
        if offset == 2:
            return Move.BACKWARD
        return Move.LEFT  # offset == 3

    def move_mouse(self) -> Move:
        """Moves the mouse one step and returns the move made"""
        next_dir = self.next_direction()
        move = self.next_move(next_dir)

        # update internal position and direction
        row, col = self._mouse_pos
        self._mouse_pos = (row + next_dir.dr, col + next_dir.dc)
        self._mouse_dir = next_dir

        return move


def rc_to_xy(position: tuple[int, int], num_rows: int) -> tuple[int, int]:
    """Convert from (row, col) to (x, y) coordinates"""
    return position[1], num_rows - 1 - position[0]


def display_walls(maze: Maze) -> None:
    position = maze.mouse_position
    for direction in Direction:
        if maze.is_wall(position, direction):
            x, y = rc_to_xy(position, maze.height)
            if direction == Direction.NORTH:
                API.setWall(x, y, "n")
            elif direction == Direction.EAST:
                API.setWall(x, y, "e")
            elif direction == Direction.SOUTH:
                API.setWall(x, y, "s")
            elif direction == Direction.WEST:
                API.setWall(x, y, "w")


def display_dists(maze: Maze) -> None:
    dists = maze.dists
    for row_idx, row in enumerate(dists):
        for col_idx, dist in enumerate(row):
            if dist != -1:
                API.setText(*rc_to_xy((row_idx, col_idx), maze.height), dist)


def log(string) -> None:
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()


def update_walls(maze: Maze) -> None:
    position = maze.mouse_position
    direction = maze.mouse_direction
    if API.wallFront():
        maze.add_wall(position, direction)
    if API.wallLeft():
        maze.add_wall(position, direction.rotate(-1))
    if API.wallRight():
        maze.add_wall(position, direction.rotate(1))


def main():
    # the API uses (x, y) cartesian coordinates
    # the Maze class uses (r, c) coordinates
    width = API.mazeWidth()
    height = API.mazeHeight()
    assert width and height

    maze = Maze(width, height)

    while True:
        # mouse goes back and forth between start and centre
        # if maze.goal == (row, col):
        #     if maze.goal == start:
        #         maze.goal = height // 2, width // 2
        #     else:
        #         log("Centre reached")
        #         maze.goal = start

        update_walls(maze)
        maze.floodfill()

        # display walls and distances for debugging
        display_walls(maze)
        display_dists(maze)

        move = maze.move_mouse()

        # move the actual mouse
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


if __name__ == "__main__":
    main()
