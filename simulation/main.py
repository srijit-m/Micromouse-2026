import API
import sys
from collections import deque
from enum import Enum
from typing import Optional


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

    @property
    def left(self):
        return self.rotate(-1)

    @property
    def right(self):
        return self.rotate(1)


class Move(Enum):
    FORWARD = 0
    RIGHT = 1
    BACKWARD = 2
    LEFT = 3


class Mouse:
    def __init__(self, start: tuple[int, int], direction: Direction) -> None:
        self._position = start
        self._direction = direction

    @property
    def position(self) -> tuple[int, int]:
        return self._position

    @property
    def direction(self) -> Direction:
        return self._direction

    def turn_left(self) -> None:
        self._direction = self._direction.left

    def turn_right(self) -> None:
        self._direction = self._direction.right

    def move_forward(self) -> None:
        row, col = self._position
        dr, dc = self._direction.dr, self._direction.dc
        self._position = (row + dr, col + dc)

    def apply_move(self, move: Move) -> None:
        if move == Move.FORWARD:
            self.move_forward()
        elif move == Move.RIGHT:
            self.turn_right()
            self.move_forward()
        elif move == Move.BACKWARD:
            self.turn_right()
            self.turn_right()
            self.move_forward()
        elif move == Move.LEFT:
            self.turn_left()
            self.move_forward()


class Maze:
    def __init__(self, width: int, height: int) -> None:
        self._height = height
        self._width = width
        self._goal = (height // 2, width // 2)  # assume odd maze size for simplicity

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
    def dists(self) -> list[list[int]]:
        return self._dists

    def get_wall(
        self, position: tuple[int, int], direction: Direction
    ) -> tuple[list[list[bool]], int, int]:
        """Returns the appropriate wall matrix and the indices for the wall
        adjacent to the specified position

        Returns:
            A tuple (walls, row_idx, col_idx), where the wall is accessed by
            walls[row_idx][col_idx]
        """
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

    def check_bounds(self, position: tuple[int, int]) -> bool:
        """Return whether the position is within the bounds of the maze"""
        row, col = position
        return 0 <= row < self.height and 0 <= col < self.width

    def add_wall(self, position: tuple[int, int], direction: Direction) -> None:
        if self.check_bounds(position):
            walls, r, c = self.get_wall(position, direction)
            walls[r][c] = True

    def is_wall(self, position: tuple[int, int], direction: Direction) -> bool:
        if self.check_bounds(position):
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
        for direction in Direction:
            neighbour = row + direction.dr, col + direction.dc
            if self.check_bounds(neighbour):  # within bounds
                if not self.is_wall(position, direction):  # not blocked by a wall
                    neighbours.append((*neighbour, direction))
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

    def next_direction(self, position: tuple[int, int]) -> Direction:
        """Returns the direction to move, based on the current position.

        Assumes that distances have been calculated already
        """
        dists = self.dists
        min_dist = dists[position[0]][position[1]]
        next_dir = Direction.NORTH
        # find neighbouring cell with lowest distance from goal
        for nr, nc, direction in self.neighbours(position):
            dist = dists[nr][nc]
            if dist != -1 and dist < min_dist:
                min_dist = dist
                next_dir = direction
        return next_dir

    def next_move(self, direction: Direction, target: Direction) -> Move:
        """Returns the required move to make based on the current direction and
        the target direction"""
        # for simplicity, take advantage of the fact that the Direction enum is ordered clockwise
        dirs = list(Direction)
        offset = (dirs.index(target) - dirs.index(direction)) % len(dirs)

        if offset == 0:
            return Move.FORWARD
        if offset == 1:
            return Move.RIGHT
        if offset == 2:
            return Move.BACKWARD
        return Move.LEFT  # offset == 3


def rc_to_xy(position: tuple[int, int], num_rows: int) -> tuple[int, int]:
    """Convert from (row, col) to (x, y) coordinates"""
    return position[1], num_rows - 1 - position[0]


def display_walls(maze: Maze, mouse: Mouse) -> None:
    """Display the walls at the mouse's current position"""
    for direction in Direction:
        if maze.is_wall(mouse.position, direction):
            x, y = rc_to_xy(mouse.position, maze.height)
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


def update_walls(maze: Maze, mouse: Mouse) -> None:
    """Update the known walls in the maze at the mouse's current position"""
    position = mouse.position
    direction = mouse.direction
    if API.wallFront():
        maze.add_wall(position, direction)
    if API.wallLeft():
        maze.add_wall(position, direction.left)
    if API.wallRight():
        maze.add_wall(position, direction.right)


def main():
    width = API.mazeWidth()
    height = API.mazeHeight()
    assert width and height

    start = (height - 1, 0)
    direction = Direction.NORTH

    maze = Maze(width, height)
    mouse = Mouse(start, direction)

    while True:
        # mouse goes back and forth between start and centre
        if mouse.position == maze.goal:
            if maze.goal == start:
                log("Start reached")
                maze.goal = height // 2, width // 2
            else:
                log("Centre reached")
                maze.goal = start

        # update walls and distances
        update_walls(maze, mouse)
        maze.floodfill()

        # display walls and distances for debugging
        display_walls(maze, mouse)
        display_dists(maze)

        # determine next move
        target_dir = maze.next_direction(mouse.position)
        move = maze.next_move(mouse.direction, target_dir)

        # update internal mouse state
        mouse.apply_move(move)

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