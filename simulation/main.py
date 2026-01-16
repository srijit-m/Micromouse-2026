import API
import sys
from collections import deque
from enum import Enum

ROW_NUM = 5
COLUMN_NUM = 5

ROW_INDEX = 0
COLUMN_INDEX = 1

#For checking floodfill points
#NORTH_INDEX = 0
#EAST_INDEX = 1
#SOUTH_INDEX = 2
#WEST_INDEX = 3

class Direction(Enum):
    NORTH = (-1, 0)
    SOUTH = (1, 0)
    EAST = (0, 1)
    WEST = (0, -1)

def map_for_simulator(floodfill_distances):
    """Return a new 2D list flipped vertically so it matches simulator coordinates."""
    ROW_NUM = len(floodfill_distances)
    COLUMN_NUM = len(floodfill_distances[0])
    
    sim_array = []
    for r in range(ROW_NUM):
        sim_array.append(floodfill_distances[ROW_NUM - 1 - r][:])  # take a copy of the row, flipped vertically
    
    return sim_array

def check_bounds(row, col, direction):
    """Checks if a neighbouring cell is within the bounds of the maze"""
    (new_row, new_col) = (row+direction.value[ROW_INDEX], col+direction.value[COLUMN_INDEX])
    if new_row < 0 or new_row > ROW_NUM-1:
        return False
    if new_col < 0 or new_col > COLUMN_NUM-1:
        return False  
    return (new_row, new_col)

def can_move(row, col, direction, hor_walls, vert_walls):
    """Checks if the neighbouring cell can be accessed (i.e. there are no walls)"""
    if (direction == Direction.NORTH):
        #For North, we are looking at a horizontal wall
        if (hor_walls[row][col] == False):
            #No walls in the way
            return True
        return False
    if (direction == Direction.EAST):
        #For east, we deal with vertical walls
        if (vert_walls[row][col+1] == False):
            return True
        return False
    if (direction == Direction.SOUTH):
        #For south we deal with a horizontal wall
        if (hor_walls[row+1][col] == False):
            return True
        return False
    if (direction == Direction.WEST):
        #For west we deal with a vertical wall
        if (vert_walls[row][col] == False):
            return True
        return False

def compute_left_wall_direction(direction):
    return {
        Direction.NORTH: Direction.WEST,
        Direction.WEST: Direction.SOUTH,
        Direction.SOUTH: Direction.EAST,
        Direction.EAST: Direction.NORTH,
    }[direction]

def compute_right_wall_direction(direction):
    return {
        Direction.NORTH: Direction.EAST,
        Direction.EAST: Direction.SOUTH,
        Direction.SOUTH: Direction.WEST,
        Direction.WEST: Direction.NORTH,
    }[direction]


class Maze:
    def __init__(self, row_num, col_num):
        self._row_num = row_num
        self._col_num = col_num
        #Make lists for horizontal and vertical walls
        self.hor_walls = [[] for i in range(row_num+1)]
        self.vert_walls = [[] for i in range(row_num)]
        #Setup list for floodfill distances
        self.floodfill_distances = [[] for i in range(row_num)]
        #Initialising an empty queue for the floodfill algorithm
        self.dq = deque()
    
    def initialise_wall_variables(self, hor_walls, vert_walls):
        """Populating the wall variables with booleans of true and false.
        Note that we already know that the outer walls are filled up"""
        #Horizontal walls first
        for i in range(ROW_NUM+1):
            for j in range(ROW_NUM):
                if i==0 or i == ROW_NUM:
                    hor_walls[i].append(True)
                else:
                    hor_walls[i].append(False)
        #Vertical walls now
        for i in range(ROW_NUM):
            for j in range(ROW_NUM+1):
                if j == 0 or j == ROW_NUM:
                    vert_walls[i].append(True)
                else:
                    vert_walls[i].append(False)
        
        #Now we have walls on the boundary
    def initialise_floodfill_nums(self, floodfill_distances, row_num, col_num):
        """For the floodfill distance 2D array, only the centre goal cell is set to 0. The 
        other cells are set to false/blank for now to show that they are not updated yet"""
        for i in range(row_num):
            for j in range(col_num):
                #Showing uninitialised distances
                floodfill_distances[i].append(-1)
        
        floodfill_distances[row_num//2][col_num//2] = 0
    def reset_floodfill_distances(self, floodfill_distances, row_num, col_num):
        """Here we want to reset the floodfill algorithm every time we hit a wall. 
        For an already intialised array, we don't need to append elements so we can just
        set the array values."""
        for i in range(row_num):
            for j in range(col_num):
                #Showing uninitialised distances
                floodfill_distances[i][j] = -1    
        floodfill_distances[row_num//2][col_num//2] = 0
        self.dq= deque()
    
    def update_walls(self, row, col, direction):
        """Function to update horizontal or vertical walls"""
        if (direction == Direction.NORTH):
            #We have got a horizontal wall
            self.hor_walls[row][col] = True
            API.setWall(col, ROW_NUM-row-1, "n")
        elif (direction == Direction.EAST):
            #Vertical wall here
            self.vert_walls[row][col+1] = True
            API.setWall(col, ROW_NUM-row-1, "e")
        elif (direction == Direction.SOUTH):
            #Horizontal wall here
            self.hor_walls[row+1][col] = True
            API.setWall(col, ROW_NUM-row-1, "s")
        elif (direction == Direction.WEST):
            #Vertical wall here
            self.vert_walls[row][col] = True
            API.setWall(col, ROW_NUM-row-1, "w")


      
    def calculate_floodfill_distances(self, row_num, col_num, dq, floodfill_distances, hor_walls, vert_walls):
        #Add the goal cell to the queue
        centre_point = (row_num//2, col_num//2)
        #print(f"Row: {centre_point[0]}, Column: {centre_point[1]}")
        dq.append(centre_point)
        while dq:
            #Carry out the floodfill algorithm
            #Pop the zero cell and save its row and column
            (row, col) = dq.popleft()
            #Now check the boxes north, east, south and west of the point
            #Note that in this for loop False + 1 = 1 (cool thing in python)
            for direction in [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]:
                #First check if the cell is unvisited and valid
                bounds_result = check_bounds(row, col, direction)
                if bounds_result == False:
                    continue
                elif floodfill_distances[bounds_result[ROW_INDEX]][bounds_result[COLUMN_INDEX]] != -1:
                    #This cell has already been visited
                    continue
                else:
                    #Now this is a valid cell, we now need to check if the cell is accessible
                    #Here use a can_move function that takes the current cell and the direction
                    if can_move(row, col, direction, hor_walls, vert_walls):
                        #Here, if the function returns True, we can update floodfill distances
                        #n_row stands for neighbour row, n_col stands for neighbour col
                        (n_row, n_col) = (row+direction.value[ROW_INDEX], col+direction.value[COLUMN_INDEX])
                        floodfill_distances[n_row][n_col] = floodfill_distances[row][col]+1
                        dq.append((n_row, n_col))

#Creating a mouse class to implement floodfill dynamically
class Mouse:
    def __init__(self, row_pos, col_pos, direction, maze):
        self.row_pos = row_pos
        self.col_pos = col_pos
        #Note that direction will be one of our direction Enums
        self.direction = direction
        self.maze = maze
    def move_forward(self):
        self.row_pos += self.direction.value[0]
        self.col_pos += self.direction.value[1]
        API.moveForward()

    def print_coords(self):
        """This function is solely for debugging"""
        print(f"row: {self.row_pos}, col: {self.col_pos}")
        print(f"Direction {self.direction}")
    def turn_right(self):
        if self.direction == Direction.NORTH:
            self.direction = Direction.EAST
        elif self.direction == Direction.EAST:
            self.direction = Direction.SOUTH
        elif self.direction == Direction.SOUTH:
            self.direction = Direction.WEST
        elif self.direction == Direction.WEST:
            self.direction = Direction.NORTH
        API.turnRight()
    def turn_left(self):
        if self.direction == Direction.NORTH:
            self.direction = Direction.WEST
        elif self.direction == Direction.EAST:
            self.direction = Direction.NORTH
        elif self.direction == Direction.SOUTH:
            self.direction = Direction.EAST
        elif self.direction == Direction.WEST:
            self.direction = Direction.SOUTH
        API.turnLeft()
    
    def sense_walls(self):
        """This function makes the micromouse sense walls directly in front, to the left and 
        to the right"""
        if (API.wallLeft()):
            #Update this in the wall array
            #Find where the wall actually is (n,e,s,w)
            wall_dir = compute_left_wall_direction(self.direction)
            self.maze.update_walls(self.row_pos, self.col_pos, wall_dir)
        if (API.wallRight()):
            wall_dir = compute_right_wall_direction(self.direction)
            self.maze.update_walls(self.row_pos, self.col_pos, wall_dir)
            
        if (API.wallFront()):
            self.maze.update_walls(self.row_pos, self.col_pos, self.direction)
         

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main():
    log("Running... the sim")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "abc")

    #Initialise details of the maze
    maze = Maze(ROW_NUM, COLUMN_NUM)
    maze.initialise_wall_variables(maze.hor_walls, maze.vert_walls)
    maze.initialise_floodfill_nums(maze.floodfill_distances, maze._row_num, maze._col_num)   
    maze.calculate_floodfill_distances(maze._row_num, maze._col_num, maze.dq, maze.floodfill_distances, maze.hor_walls, maze.vert_walls)
    for row in maze.floodfill_distances:
        print(" ".join(str(val) for val in row))

    for r in range(ROW_NUM):
        for c in range(COLUMN_NUM):
            API.setText(c, ROW_NUM-1-r, maze.floodfill_distances[r][c])

    mouse = Mouse(ROW_NUM-1, 0, Direction.NORTH, maze)   
    mouse.sense_walls()
    mouse.move_forward()
    mouse.sense_walls()
    mouse.move_forward()
    mouse.sense_walls()
    mouse.turn_right()
    mouse.move_forward()
    mouse.sense_walls()
    mouse.maze.reset_floodfill_distances(mouse.maze.floodfill_distances, mouse.maze._row_num, mouse.maze._col_num)
    mouse.maze.calculate_floodfill_distances(mouse.maze._row_num, mouse.maze._col_num, mouse.maze.dq, mouse.maze.floodfill_distances, mouse.maze.hor_walls, mouse.maze.vert_walls)
    API.clearAllText()
    for row in mouse.maze.floodfill_distances:
        print(" ".join(str(val) for val in row))
    for r in range(ROW_NUM):
        for c in range(COLUMN_NUM):
            API.setText(c, ROW_NUM-1-r, mouse.maze.floodfill_distances[r][c])
    mouse.turn_right()
    mouse.move_forward()
    mouse.sense_walls()
    mouse.turn_left()
    mouse.move_forward()
    mouse.sense_walls()
    mouse.turn_left()
    mouse.move_forward()
    mouse.sense_walls()
    mouse.print_coords()
    
        

if __name__ == "__main__":
    main()
