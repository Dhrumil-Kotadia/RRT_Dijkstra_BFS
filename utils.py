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
    # 4 step movement
    x = curr_node[0]
    y = curr_node[1]
    
    next_steps = [
    (x+1, y), (x-1, y), (x, y+1), (x, y-1),(x+1, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1)]
    
    valid_neighbors = []
    for step in next_steps:
        if 0 <= step[0] < len(grid) and 0 <= step[1] < len(grid[0]):
            if grid[step[0]][step[1]] == False:
                valid_neighbors.append(step)
    return valid_neighbors

def bfs(grid, start, goal):
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

def rrt(grid, start, goal, max_iters=5000, step_size=1):
    if grid[goal[0]][goal[1]] or grid[start[0]][start[1]]:
        return None

    def is_free(p):
        x, y = p
        return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and not grid[x][y]

    def distance(p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def steer(from_node, to_node):
        vec = (to_node[0] - from_node[0], to_node[1] - from_node[1])
        dist = math.hypot(*vec)
        if dist <= step_size:
            return to_node
        else:
            scale = step_size / dist
            new_point = (int(round(from_node[0] + vec[0] * scale)),
                         int(round(from_node[1] + vec[1] * scale)))
            return new_point

    def collision_free(p1, p2):
        # Bresenham’s Line Algorithm for grid collision checking
        x0, y0 = p1
        x1, y1 = p2
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if not is_free((x, y)):
                    return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if not is_free((x, y)):
                    return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        return is_free((x, y))

    tree = {tuple(start): None}
    nodes = [tuple(start)]

    for _ in range(max_iters):
        print("Iteration:", _, end="\r")
        rand_point = (random.randint(0, len(grid)-1), random.randint(0, len(grid[0])-1))
        nearest = min(nodes, key=lambda n: distance(n, rand_point))
        new_point = steer(nearest, rand_point)
        if is_free(new_point) and collision_free(nearest, new_point):
            if new_point not in tree:
                tree[new_point] = nearest
                nodes.append(new_point)
                # if new_point == tuple(goal):
                #     break
                if distance(new_point, goal) <= step_size and collision_free(new_point, goal):
                    tree[tuple(goal)] = nearest
                    break

    # Trace path back from goal to start
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