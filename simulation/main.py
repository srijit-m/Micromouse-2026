import API
import sys
from collections import deque
from enum import Enum

Position = tuple[int, int]


# (row, col) coordinate system
class Dir(Enum):
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


class Move(Enum):
    FORWARD = 0
    LEFT = 1
    RIGHT = 2


class Maze:
    def __init__(self, rows, cols) -> None:
        self._rows = rows
        self._cols = cols
        self._goal = (rows // 2, cols // 2)  # assume odd maze size for simplicity
        self._h_walls = [[False] * cols for _ in range(rows + 1)]
        self._v_walls = [[False] * (cols + 1) for _ in range(rows)]
        self._dists = [[-1] * cols for _ in range(rows)]

    @property
    def height(self):
        return self._rows

    @property
    def width(self):
        return self._cols

    def set_goal(self, row, col):
        self._goal = (row, col)

    def get_dists(self) -> list[list[int]]:
        return self._dists

    def get_wall(self, row, col, direction: Dir) -> tuple[list[list[bool]], int, int]:
        """Returns the matrix representing horizontal or vertical walls and the
        indexes corresponding to the wall at the specified position"""
        if direction == Dir.NORTH:
            return self._h_walls, row, col
        if direction == Dir.EAST:
            return self._v_walls, row, col + 1
        if direction == Dir.SOUTH:
            return self._h_walls, row + 1, col
        if direction == Dir.WEST:
            return self._v_walls, row, col

    def add_wall(self, row, col, direction) -> None:
        walls, r, c = self.get_wall(row, col, direction)
        walls[r][c] = True

    def is_wall(self, row, col, direction: Dir) -> bool:
        walls, r, c = self.get_wall(row, col, direction)
        return walls[r][c]

    def neighbours(self, row, col) -> list[tuple[int, int, Dir]]:
        """Returns the accessible neighbouring cells from the specified position,
        i.e. cells that are within bounds and not blocked by a wall.

        Returns:
            List of tuples where each tuple contains (row, col, dir)
        """
        neighbours = []
        for d in Dir:
            nr, nc = row + d.dr, col + d.dc
            if 0 <= nr < self._rows and 0 <= nc < self._cols:  # within bounds
                if not self.is_wall(row, col, d):  # not blocked by a wall
                    neighbours.append((nr, nc, d))
        return neighbours

    def floodfill(self) -> None:
        self._dists = [[-1] * self._cols for _ in range(self._rows)]
        self._dists[self._goal[0]][self._goal[1]] = 0
        q = deque([self._goal])
        while q:
            row, col = q.popleft()
            for nr, nc, _ in self.neighbours(row, col):
                if self._dists[nr][nc] == -1:  # check cell is blank
                    self._dists[nr][nc] = self._dists[row][col] + 1
                    q.append((nr, nc))


def display_dists(maze: Maze):
    dists = maze.get_dists()
    for row_idx, row in enumerate(dists):
        for col_idx, dist in enumerate(row):
            API.setText(*rc_to_xy(row_idx, col_idx, maze.height), dist)


def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()


def rc_to_xy(row, col, num_rows):
    """Convert from (row, col) to (x, y) coordinates"""
    return col, num_rows - 1 - row


def main():
    # the API uses (x, y) cartesian coordinates
    # the Maze class uses (r, c) graphics coordinates
    log("Running...")
    API.setColor(0, 0, "G")

    height = API.mazeHeight()
    width = API.mazeWidth()

    # starting position and direction
    row = height - 1
    col = 0
    dir = Dir.NORTH

    maze = Maze(height, width)

    while True:
        # check for walls
        if API.wallFront():
            maze.add_wall(row, col, Dir.NORTH)
            API.setWall(*rc_to_xy(row, col, height), "n")
        if API.wallLeft():
            maze.add_wall(row, col, Dir.WEST)
            API.setWall(*rc_to_xy(row, col, height), "w")
        if API.wallRight():
            maze.add_wall(row, col, Dir.EAST)
            API.setWall(*rc_to_xy(row, col, height), "e")

        # update distances
        maze.floodfill()
        display_dists(maze)

    log(maze.next_move(row, col))

    # while True:
    #     if not API.wallLeft():
    #         API.turnLeft()
    #     while API.wallFront():
    #         API.turnRight()
    #     API.moveForward()


if __name__ == "__main__":
    main()
