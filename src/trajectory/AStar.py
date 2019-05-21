#Imports
import numpy as np
#import seaborn as sns
import math
import matplotlib.pyplot as plt

#--------------------------------
#           GLOBAL VARIABLES  
#--------------------------------
x = 8000
y = 8000

finesse = 150
width = int(math.ceil(150/finesse))

m = math.floor(y/finesse)
n = math.floor(x/finesse)

top = (m*finesse)/2

maze = np.ones(shape = (m,n))

cmap = "GnBu"

#--------------------------------
#         AUX FUNCTIONS 
#--------------------------------

def correct_position(pos):
    if pos[0] < 0:
        pos[0] = 0
    elif pos[0] >= m:
        pos[0] = m -1

    if pos[1] < 0:
        pos[1] = 0
    elif pos[1] >= n:
        pos[1] = n -1

    return pos

def validate(obstacle):
    return obstacle[0] >= 0 and obstacle[0] < m and obstacle[1] >= 0 or obstacle[1] < n

def print_path(maze,path,start,end,name):
    final_maze = maze.copy()
    
    for p in path:
        final_maze[p[0]][p[1]] = 0.7
        
    final_maze[start[0]][start[1]] = 0.8
    final_maze[end[0]][end[1]] = 0.85
    
    #sns.heatmap(final_maze, linewidths = 0.01, cmap = cmap)
    plt.imshow(final_maze, cmap = "viridis", interpolation = "nearest")
    plt.savefig(name,dpi = 200)


def get_neighbors():
    neighbors = []
    
    for x in range(-width,width+1):
        for y in range(-width,width+1):
            neighbors.append((x,y))
    
    neighbors.remove((0,0))
    return neighbors


def heuristic(current, end):
    h = ((current.position[0] - end.position[0]) ** 2) + ((current.position[1] - end.position[1]) ** 2)
    #h = np.abs(current.position[0] - end.position[0]) + np.abs(current.position[1] - end.position[1])
    return h

def map_position(position):
    j = math.floor(position[0]/finesse)
    i = math.floor((-position[1]+top)/finesse)
    
    return int(i),int(j)

def map_inverse(matrix_pos):
    
    half = finesse/2
    
    x = (matrix_pos[1]*finesse) + half
    y = top - (matrix_pos[0]*finesse) - half
    
    return x,y

def is_blocked(obstacle, matrix_pos):
    x,y = map_inverse(matrix_pos)
    
    distance = np.sqrt(np.power(x - obstacle[0],2) + np.power(y - obstacle[1],2))
    
    return 1 if distance > 150 else 0 


def check_neighbors(obstacle):
    global maze
    center_pos = map_position(obstacle)
    
    print obstacle
    print center_pos

    if not validate(center_pos):
        print 'Out of bounds'
        break
    
    maze[center_pos[0]][center_pos[1]] = 0.3
    
    for new_position in get_neighbors():
        #Get node position
        pos = (center_pos[0] + new_position[0], center_pos[1] + new_position[1])

        if pos[0] > (len(maze) - 1) or pos[0] < 0 or pos[1] > (len(maze[0]) - 1) or pos[1] < 0:
            print 'continue'
            continue
            
        res = is_blocked(obstacle, pos) 
        
        print 'Position {} is {}'.format(pos,res)
            
        maze[pos[0]][pos[1]] = res  

def final_path(path):
    trajectory = []

    for p in path:
        trajectory.append(map_inverse(p))
    
    return trajectory

#------------------------------------
#         A-STAR IMPLEMENTATION 
#------------------------------------
class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    print "A STAR FUNCTION"

    print 'START: {}'.format(start)

    print 'END: {}'.format(end)

    start_node = Node(None, position = start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, position = end)
    end_node.g = end_node.h = end_node.f = 0
    
    open_list = []
    closed_list = []
    
    open_list.append(start_node)
    
    while len(open_list) > 0:
        
        current_node = open_list[0]
        current_index = 0
        
        #Find the best node
        for index,item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                
        #Pop current node and add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        
        #Finished
        if current_node == end_node:
            path = []
            current = current_node
            
            while current is not None:
                path.append(current.position)
                current = current.parent
                
            return path[::-1] #Return reversed path
        
        #Generate children
        children = []
        
        for new_position in [(0,-1),(0,1),(1,0),(-1,0),(-1,1),(-1,-1),(1,-1),(1,1)]:
            
            #Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[0]) - 1) or node_position[1] < 0:
                continue
                
            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 1:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)
            

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = heuristic(child,end_node)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

#--------------------------------
#          MAIN 
#--------------------------------
def run_astar(obstacles, start, end):
    start_pos = map_position(start)
    end_pos = map_position(end)

    start_pos = correct_position(start_pos)
    end_pos = correct_position(end_pos)

    for index,obstacle in enumerate(obstacles):
        print '\nObstacle {}'.format(index)
        check_neighbors(obstacle)

    print_path(maze,[],start_pos,end_pos, 'og.png')

    path = astar(maze, start_pos, end_pos)

    print_path(maze,path,start_pos,end_pos, 'solved.png')

    return final_path(path)
