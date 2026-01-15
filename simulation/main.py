import API
import sys
from collections import deque
from enum import Enum

ROW_NUM = 9
COLUMN_NUM = 9


def get_neighbour_coords(row, col):
        """This function returns the north, east, south and west coordinates of the points relative
        to a point popped out of the queue.
        Returns: A list of 4 tuples of the form (row, col)"""
        neighbour_points = []
        neighbour_points.append((row+Direction.NORTH.value[0], col+Direction.NORTH.value[1]))
        neighbour_points.append((row+Direction.EAST.value[0], col+Direction.EAST.value[1]))
        neighbour_points.append((row+Direction.SOUTH.value[0], col+Direction.SOUTH.value[1]))
        neighbour_points.append((row+Direction.WEST.value[0], col+Direction.WEST.value[1]))
        return neighbour_points
class Direction(Enum):
    NORTH = (-1, 0)
    SOUTH = (1, 0)
    EAST = (0, 1)
    WEST = (0, -1)

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
                floodfill_distances[i].append(False)
        
        floodfill_distances[row_num//2][col_num//2] = 0
    def reset_floodfill_distances(self, floodfill_distances, row_num, col_num):
        """Here we want to reset the floodfill algorithm every time we hit a wall. 
        For an already intialised array, we don't need to append elements so we can just
        set the array values."""
        for i in range(row_num):
            for j in range(col_num):
                #Showing uninitialised distances
                floodfill_distances[i][j] = False    
        floodfill_distances[row_num//2][col_num//2] = 0
      
    def calculate_floodfill_distances(self, row_num, col_num, dq, floodfill_distances, hor_walls, vert_walls):
        #Add the goal cell to the queue
        centre_point = (row_num//2, col_num//2)
        dq.append(centre_point)
        while dq:
            #Carry out the floodfill algorithm
            #Pop the zero cell and save its row and column
            (row, col) = dq.pop()
            print(f"Row: {row}, Col: {col}")
            #Now check the boxes north, east, south and west of the point
            neighbour_points = get_neighbour_coords(row, col)
            #Note that in this for loop False + 1 = 1 (cool thing in python)
            for i in range(4):
                if (i == 0):
                    #Looking at the north cell so thus horizontal walls
                    if (hor_walls[row][col] == False):
                        floodfill_distances[neighbour_points[i][0]][neighbour_points[i][1]] = floodfill_distances[row][col] +1
                        #Add cell to queue
                        dq.append(neighbour_points[i])
                elif (i == 1):
                    #Looking at the east cell so thus vertical walls
                    if (vert_walls[row][col+1] == False):
                        #There are no walls
                        floodfill_distances[neighbour_points[i][0]][neighbour_points[i][1]] = floodfill_distances[row][col] +1
                        dq.append(neighbour_points[i])
                elif (i == 2):
                    #Looking at the south cell, so horizontal walls
                    if (hor_walls[row+1][col] == False):
                        floodfill_distances[neighbour_points[i][0]][neighbour_points[i][1]] = floodfill_distances[row][col] +1
                        dq.append(neighbour_points[i])
                elif (i == 3):
                    #Looking at the west cell, so thus vertical walls
                    if (vert_walls[row][col] == False):
                        floodfill_distances[neighbour_points[i][0]][neighbour_points[i][1]] = floodfill_distances[row][col] +1
                        dq.append(neighbour_points[i])




                    
    
        



def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

def main():
    log("Running...")
    API.setColor(0, 0, "G")
    API.setText(0, 0, "abc")

    #Initialise details of the maze
    maze = Maze(ROW_NUM, COLUMN_NUM)
    maze.initialise_wall_variables(maze.hor_walls, maze.vert_walls)
    maze.initialise_floodfill_nums(maze.floodfill_distances, maze._row_num, maze._col_num)


    #Wall follower algorithm    
    while True:
        if not API.wallLeft():
            API.turnLeft()
        while API.wallFront():
            API.turnRight()
        API.moveForward()

if __name__ == "__main__":
    main()
