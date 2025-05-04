import numpy as np
import time
from pal.products.qcar import QCar
import matplotlib.pyplot as plt
import heapq
import math

node_coordinates = {
    0: (0.0, 0.0), 1: (1.0 , 0.0) , 2: (1.0 , 8.0)
}

def euclidean(p1, p2):
    return round(math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2), 3)

graph = {
    0: [1], 1: [2] , 2: [1]
}

weighted_graph = {k: {n: euclidean(node_coordinates[k], node_coordinates[n]) for n in v} for k, v in graph.items()}

def dijkstra(graph, start, goal):
    unvisited = set(graph.keys())
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    previous = {node: None for node in graph}
    
    while unvisited:
        current = min(unvisited, key=lambda node: distances[node])
        if current == goal:
            break
        unvisited.remove(current)

        for neighbor, weight in graph[current].items():
            new_distance = distances[current] + weight
            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                previous[neighbor] = current

    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = previous[current]
    return path[::-1]

def generate_path(graph, node_sequence):
    full_path = []
    for i in range(len(node_sequence) - 1):
        segment = dijkstra(graph, node_sequence[i], node_sequence[i+1])
        full_path.extend(segment[:-1])
    full_path.append(node_sequence[-1])
    return full_path

def move_qcar_along_path(hqcar, path, speed=0.05):
    print("Starting QCar movement along the planned path...")
    for i in range(path.shape[1] - 1):
        x, y = path[0, i], path[1, i]
        next_x, next_y = path[0, i+1], path[1, i+1]

        throttle = speed
        dx, dy = next_x - x, next_y - y
        steering = np.arctan2(dy, dx)  

        hqcar.write(throttle, steering, np.array([0]*6 + [1, 1]))
        print(f"Moving to ({next_x:.2f}, {next_y:.2f})")
        time.sleep(1)

    hqcar.write(0.0, 0.0, np.array([0]*6 + [1, 1]))
    print("QCar has reached the final destination.")

def setup(initialPosition=[-0.031, 1.311, 0.000], initialOrientation=[0, 0, -np.pi/2], path=None):
    with QCar(frequency=200) as hqcar:
        print("Initializing QCar...")
        if path is not None:
            move_qcar_along_path(hqcar, path)
        else:
            print("No valid path found.")

def main():
    node_sequence = [0 , 1 , 2 , 0 , 1 , 0 , 1 , 2 , 0 , 1, 0 , 1,  0 , 2 , 0 , 2 , 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,0, 1,
                    0 ,1 , 0, 1     , 0 , 1 , 0 , 2 , 0 , 1, 2,0,1 ,      0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 ,  2  , 0,1,0,1,0,1 ,   2,
                     0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,   2  , 0,1,2  , 0,1,  0,1]  
    
    path = generate_path(weighted_graph, node_sequence)
    print("Planned path (nodes):", path)
    path_coordinates = np.array([node_coordinates[node] for node in path]).T
    setup(path=path_coordinates)

if __name__ == "__main__":
    main()
