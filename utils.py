import math
import matplotlib.pyplot as plt
import numpy as np
import random
import heapq

def read_grid_from_file(file_path):
    
    grid = []
    with open(file_path, 'r') as file:
        for line in file:
            # Strip the newline character and split by spaces
            grid.append(line.strip().split())
    
    # Convert the grid to a 2D list of bools
    for i in range(len(grid)):
        string = grid[i][0]
        row = []
        for j in range(len(string)):
            if string[j] == '.':
                row.append(False)
            elif string[j] == 'X':
                row.append(True)
        grid[i] = row
            
    return grid

def calculate_path_length(path):
    
    length = 0
    for i in range(1, len(path)):
        length += math.sqrt(sum((path[i][j] - path[i-1][j])**2 for j in range(len(path[i]))))
    
    return length

def get_valid_neighbors(grid, curr_node):
    x, y = curr_node
    directions = [
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (-1, -1), (1, -1), (-1, 1)
    ]

    valid_neighbors = []

    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]):
            if not grid[nx][ny]:
                if abs(dx) + abs(dy) == 2:
                    if grid[x][ny] or grid[nx][y]:
                        continue
                valid_neighbors.append((nx, ny))

    return valid_neighbors


def bfs(grid, start, goal):
    """
    Breadth-First Search (BFS) algorithm for pathfinding.
    Args:
        grid (list): 2D list representing the grid (True for obstacles, False for free space).
        start (tuple): Starting point (x, y).
        goal (tuple): Goal point (x, y).
    Returns:
        list: List of tuples representing the path from start to goal, or None if no path is found.
    """
    if grid[goal[0]][goal[1]] == True or grid[start[0]][start[1]] == True:
        return None
    visited = [[False] * len(grid[0]) for _ in range(len(grid))]
    parent = [[0] * len(grid[0]) for _ in range(len(grid))]
    queue = []
    queue.append(start)
    visited[start[0]][start[1]] = True
    parent[start[0]][start[1]] = -1

    while(len(queue) != 0):

        curr_node = queue.pop(0)
        neighbors = get_valid_neighbors(grid, curr_node)
        for neighbor in neighbors:
            if not visited[neighbor[0]][neighbor[1]]:
                queue.append(neighbor)
                visited[neighbor[0]][neighbor[1]] = True
                parent[neighbor[0]][neighbor[1]] = curr_node
                if neighbor[0] == goal[0] and neighbor[1] == goal[1]:
                    path = []
                    while parent[neighbor[0]][neighbor[1]] != -1:
                        path.append(neighbor)
                        neighbor = parent[neighbor[0]][neighbor[1]]
                    path.append(start)
                    path.reverse()
                    return path
    return None


def dijkstra(grid, start, goal):
    """
    Dijkstra's algorithm for pathfinding.
    Args:
        grid (list): 2D list representing the grid (True for obstacles, False for free space).
        start (tuple): Starting point (x, y).
        goal (tuple): Goal point (x, y).
    Returns:
        list: List of tuples representing the path from start to goal, or None if no path is found.
    """
    if grid[goal[0]][goal[1]] == True or grid[start[0]][start[1]] == True:
        return None
    visited = [[False] * len(grid[0]) for _ in range(len(grid))]
    parent = [[False] * len(grid[0]) for _ in range(len(grid))]
    distances = [[float('inf')] * len(grid[0]) for _ in range(len(grid))]
    queue = []
    
    heapq.heappush(queue,(0,start))
    parent[start[0]][start[1]] = -1
    distances[start[0]][start[1]] = 0

    while(len(queue) > 0):
        dist, curr_node = heapq.heappop(queue)
        visited[curr_node[0]][curr_node[1]] = True
        neighbors = get_valid_neighbors(grid, curr_node)

        for neighbor in neighbors:
            if visited[neighbor[0]][neighbor[1]] == False:
                edge_distance = 1
                node_distance = edge_distance + distances[curr_node[0]][curr_node[1]]

                if node_distance < distances[neighbor[0]][neighbor[1]]:
                    distances[neighbor[0]][neighbor[1]] = node_distance
                    parent[neighbor[0]][neighbor[1]] = curr_node
                    heapq.heappush(queue,(node_distance, neighbor))

                if neighbor[0] == goal[0] and neighbor[1] == goal[1]:
                    path = []
                    path.append(goal)
                    final = goal
                    while(parent[final[0]][final[1]] != -1):
                        path.append(parent[final[0]][final[1]])
                        final = parent[final[0]][final[1]]
                    path.append(start)
                    path.reverse()
                    return path
    
    return None

def rrt(grid, start, goal, max_iters=5000):
    """
    Rapidly-exploring Random Tree (RRT) path planning algorithm.
    Args:
        grid (list): 2D list representing the grid (True for obstacles, False for free space).
        start (tuple): Starting point (x, y).
        goal (tuple): Goal point (x, y).
        max_iters (int): Maximum iterations for RRT.
    Returns:
        list: List of tuples representing the path from start to goal, or None if no path is found.
    """
    if grid[goal[0]][goal[1]] or grid[start[0]][start[1]]:
        return None

    def is_free(p):
        """Check if a point is within the grid and not an obstacle."""
        x, y = p
        return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and not grid[x][y]

    def distance(p1, p2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def get_best_neighbor(from_node, sample_point):
        """Find the best neighbor of a node that is closest to the sample point."""
        neighbors = get_valid_neighbors(grid, from_node)
        if not neighbors:
            return None
        return min(neighbors, key=lambda n: distance(n, sample_point))

    tree = {tuple(start): None}
    nodes = [tuple(start)]

    for _ in range(max_iters):
        print("Iteration:", _, end="\r")
        rand_point = (random.randint(0, len(grid) - 1), random.randint(0, len(grid[0]) - 1))
        nearest = min(nodes, key=lambda n: distance(n, rand_point))
        new_point = get_best_neighbor(nearest, rand_point)

        if new_point and new_point not in tree:
            tree[new_point] = nearest
            nodes.append(new_point)

            if distance(new_point, goal) <= 1.5 and is_free(goal):
                if tuple(goal) not in tree:
                    tree[tuple(goal)] = new_point
                    nodes.append(goal)
                break
    
    if tuple(goal) in tree:
        path = []
        node = tuple(goal)
        while node is not None:
            path.append(node)
            node = tree[node]
        path.reverse()
        return path

    return None



def plot_grid(grid, path=None, start=None, goal=None):
    
    # Create a color map for the grid
    cmap = plt.cm.get_cmap('Greys')
    cmap.set_under(color='white') # Free space color
    cmap.set_over(color='black')
    
    # Obstacle color
    grid_array = np.asarray(grid)
    fig, ax = plt.subplots()

    # Plot the grid with respect to the upper left-hand corner
    ax.matshow(grid_array, cmap=cmap, vmin=0.1, vmax=1.0)
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1)
    ax.set_xticks(np.arange(-0.5, len(grid[0]), 1))
    ax.set_yticks(np.arange(-0.5, len(grid), 1))
    ax.set_xticklabels(range(0, len(grid[0])+1))
    ax.set_yticklabels(range(0, len(grid)+1))
    
    # Plot the path with direction arrows
    if path:
        for i in range(len(path) - 1):
            start_y, start_x = path[i]
            end_y, end_x = path[i + 1]
            # start_y = len(grid) - start_y - 1
            # end_y = len(grid) - end_y - 1
            ax.arrow(start_x, start_y, end_x - start_x, end_y - start_y,head_width=0.3, head_length=0.3, fc='blue', ec='blue')
        # Plot the last point in the path
        ax.plot(path[-1][1], path[-1][0], 'b.')
    
    # Plot the start and goal points
    if start:
        ax.plot(start[1], start[0], 'go') # Start point in green
    if goal:
        ax.plot(goal[1], goal[0], 'ro') # Goal point in red
    
    return fig