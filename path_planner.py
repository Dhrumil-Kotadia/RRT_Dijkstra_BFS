import matplotlib.pyplot as plt
import numpy as np
import utils
import json
from time import time
import tracemalloc

if __name__ == "__main__":
    
    grid_path = "/media/storage/lost+found/WPI/Sem4/Advanced_Nav/Project1/grid3.txt"
    grid = utils.read_grid_from_file(grid_path)
    start_goal_path = "/media/storage/lost+found/WPI/Sem4/Advanced_Nav/Project1/startgoal3.json"

    with open(start_goal_path, 'r') as file:
        start_goal = json.load(file)
    
    for key in start_goal:
        start = start_goal[key][0]
        goal = start_goal[key][1]
        
        ################## BFS ##################
        start_time = time()
        tracemalloc.start()
        path = utils.bfs(grid, start, goal)
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        end_time = time()
        
        print("-------------------------------------")
        print(f"Start: {start}, Goal: {goal}")
        print("-------------------------------------")
        print(f"Memory usage BFS: {current / 10**6}MB; Peak: {peak / 10**6}MB")
        print(f"Time taken for BFS from {start} to {goal}: {end_time - start_time} seconds")
        
        if path is not None:
            path_length = utils.calculate_path_length(path)
            print(f"Path length: {path_length}")
        else:
            print("No path Found!!")

        fig = utils.plot_grid(grid, path, start, goal)
        plt.savefig(f"bfs_grid_3_startgoalpair_{key}.png")
        plt.close(fig)

        ################ Dijkstra ################
        start_time = time()
        tracemalloc.start()
        path = utils.dijkstra(grid, start, goal)
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        end_time = time()
        print("-------------------------------------")
        print(f"Memory usage Dijkstra: {current / 10**6}MB; Peak: {peak / 10**6}MB")
        print(f"Time taken for Dijkstra from {start} to {goal}: {end_time - start_time} seconds")
        if path is not None:
            path_length = utils.calculate_path_length(path)
            print(f"Path length: {path_length}")
        else:
            print("No path Found!!")
        print("-------------------------------------")
        fig = utils.plot_grid(grid, path, start, goal)
        plt.savefig(f"dijkstra_grid_3_startgoalpair_{key}.png")
        plt.close(fig)

        ################## RRT ##################
        start_time = time()
        tracemalloc.start()
        path = utils.rrt(grid, start, goal, max_iters=5000)
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        end_time = time()

        print("-------------------------------------")
        print(f"Memory usage RRT: {current / 10**6}MB; Peak: {peak / 10**6}MB")
        print(f"Time taken for RRT from {start} to {goal}: {end_time - start_time} seconds")

        if path is not None:
            path_length = utils.calculate_path_length(path)
            print(f"Path length: {path_length}")
        else:
            print("No path Found!!")
        print("-------------------------------------")
        print("-------------------------------------")


        fig = utils.plot_grid(grid, path, start, goal)
        plt.savefig(f"rrt_grid_3_startgoalpair_{key}.png")
        plt.close(fig)

    
