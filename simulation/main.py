import API
import sys
from collections import deque
from enum import Enum
import time


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
    TURN_LEFT = 1
    TURN_RIGHT = 2
    TURN_AROUND = 3


class Mouse:
    def __init__(self, start_pos: tuple[int, int], start_dir: Direction) -> None:
        self._start_pos = start_pos
        self._start_dir = start_dir
        self._position = start_pos
        self._direction = start_dir

    @property
    def start_position(self) -> tuple[int, int]:
        return self._start_pos

    @property
    def start_direction(self) -> Direction:
        return self._start_dir

    @property
    def position(self) -> tuple[int, int]:
        return self._position

    @property
    def direction(self) -> Direction:
        return self._direction

    def reset(self) -> None:
        self._position = self._start_pos
        self._direction = self._start_dir

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
        elif move == Move.TURN_LEFT:
            self.turn_left()
        elif move == Move.TURN_RIGHT:
            self.turn_right()
        elif move == Move.TURN_AROUND:
            self.turn_right()
            self.turn_right()


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

    @dists.setter
    def dists(self, dists) -> None:
        self._dists = dists

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

    def add_wall(self, position: tuple[int, int], direction: Direction) -> bool:
        """Add a wall to the maze and return whether the wall state changed."""
        if not self.check_bounds(position):
            return False

        walls, r, c = self.get_wall(position, direction)

        if walls[r][c]:
            return False

        walls[r][c] = True
        return True

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

    def next_direction(
        self, position: tuple[int, int], heading: Direction
    ) -> Direction:
        """Returns the direction to move, based on the current position and direction.

        When multiple neighbouring cells have the same distance from the goal,
        the order of precedence is:
        1. Forward
        2. Left
        3. Right
        4. Backward
        """
        directions = (
            heading,
            heading.left,
            heading.right,
            heading.rotate(2),
        )

        row, col = position
        current_dist = self.dists[row][col]

        for direction in directions:
            nr = row + direction.dr
            nc = col + direction.dc

            # check that the neighbour is within bounds and not blocked by a wall
            if not self.check_bounds((nr, nc)) or self.is_wall(position, direction):
                continue

            if self.dists[nr][nc] == current_dist - 1:
                return direction

        return heading


def next_move(direction: Direction, target: Direction) -> Move:
    """Returns the required move to make based on the current direction and
    the target direction"""
    # for simplicity, take advantage of the fact that the Direction enum is ordered clockwise
    dirs = list(Direction)
    offset = (dirs.index(target) - dirs.index(direction)) % len(dirs)

    if offset == 0:
        return Move.FORWARD
    if offset == 1:
        return Move.TURN_RIGHT
    if offset == 2:
        return Move.TURN_AROUND
    return Move.TURN_LEFT  # offset == 3


def update_walls(maze: Maze, mouse: Mouse) -> bool:
    """Update the known walls in the maze at the mouse's current position.

    Returns:
        Whether any new walls were added.
    """
    position = mouse.position
    direction = mouse.direction
    updated = False
    if API.wallFront():
        updated |= maze.add_wall(position, direction)
    if API.wallLeft():
        updated |= maze.add_wall(position, direction.left)
    if API.wallRight():
        updated |= maze.add_wall(position, direction.right)
    return updated


def extract_path(
    maze: Maze, start_pos: tuple[int, int], start_dir: Direction, goal: tuple[int, int]
) -> list[Direction]:
    """Extract the shortest path as a list of directions from start to goal"""
    old_goal = maze.goal
    old_dists = maze.dists
    if old_goal != goal:
        maze.goal = goal
        maze.floodfill()

    path = []
    position = start_pos
    direction = start_dir

    while position != maze.goal:
        next_dir = maze.next_direction(position, direction)
        path.append(next_dir)

        position = (position[0] + next_dir.dr, position[1] + next_dir.dc)
        direction = next_dir

    # restore old goal and distances
    if old_goal != goal:
        maze.goal = old_goal
        maze.dists = old_dists

    return path


def compress_path(path: list[Direction]) -> list[tuple[Direction, int]]:
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


def path_to_moves(
    path: list[tuple[Direction, int]], start_dir: Direction
) -> list[tuple[Move, int]]:
    moves = []
    prev_dir = start_dir
    for direction, count in path:
        move = next_move(prev_dir, direction)
        if move != Move.FORWARD:
            moves.append((move, 1))
        moves.append((Move.FORWARD, count))
        prev_dir = direction
    return moves


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


def explore_maze(maze: Maze, mouse: Mouse) -> list[tuple[Move, int]]:
    """Explore the maze and return optimal list of moves to solve maze"""
    centre = maze.height // 2, maze.width // 2
    last_path = None
    while True:
        if mouse.position == maze.goal:
            if maze.goal == mouse.start_position:
                log("Start reached")
                new_goal = centre
            else:
                log("Centre reached")
                new_goal = mouse.start_position

            path = extract_path(
                maze, mouse.start_position, mouse.start_direction, centre
            )

            if path == last_path:  # path converged
                return path_to_moves(compress_path(path), mouse.start_direction)

            last_path = path
            maze.goal = new_goal
            maze.floodfill()

        # update walls and distances
        if update_walls(maze, mouse):
            maze.floodfill()  # only recalculate if walls changed

        # display walls and distances for debugging
        display_walls(maze, mouse)
        display_dists(maze)

        # determine next move
        target_dir = maze.next_direction(mouse.position, mouse.direction)
        move = next_move(mouse.direction, target_dir)

        # update internal mouse state
        mouse.apply_move(move)

        # move the actual mouse
        execute_move(move)


def execute_move(move: Move) -> None:
    if move == Move.FORWARD:
        API.moveForward()
    elif move == Move.TURN_RIGHT:
        API.turnRight()
    elif move == Move.TURN_AROUND:
        API.turnRight()
        API.turnRight()
    elif move == Move.TURN_LEFT:
        API.turnLeft()


def log(string) -> None:
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()


def main():
    width = API.mazeWidth()
    height = API.mazeHeight()
    assert width and height

    start_pos = (height - 1, 0)
    start_dir = Direction.NORTH

    maze = Maze(width, height)
    mouse = Mouse(start_pos, start_dir)

    moves = explore_maze(maze, mouse)

    # fast run
    API.ackReset()
    mouse.reset()

    time.sleep(1)
    log("starting fast run")
    for move, count in moves:
        for _ in range(count):
            API.setColor(*rc_to_xy(mouse.position, height), "G")
            mouse.apply_move(move)
            execute_move(move)


if __name__ == "__main__":
    main()