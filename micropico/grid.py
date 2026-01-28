from micropython import const

# Directions
NORTH = const(0)
EAST  = const(1)
SOUTH = const(2)
WEST  = const(3)
DIRECTIONS = (NORTH, EAST, SOUTH, WEST)

_DELTAS = (
    (0, 1),   # NORTH
    (1, 0),   # EAST
    (0, -1),  # SOUTH
    (-1, 0),  # WEST
)

# Moves
FORWARD = const(0)
TURN_LEFT = const(1)
TURN_RIGHT = const(2)
TURN_AROUND = const(3)

def step(pos, direction, n=1):
    x, y = pos
    dx, dy = _DELTAS[direction]
    return x + n * dx, y + n * dy

def left(d):
    return (d - 1) & 3

def right(d):
    return (d + 1) & 3

def behind(d):
    return (d + 2) & 3


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
