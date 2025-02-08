#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import random
from scipy.spatial import KDTree

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0  # Total cost to reach this node

class RRTStar:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def build_kd_tree(self):
        points = [(node.x, node.y) for node in self.nodes]
        self.kd_tree = KDTree(points)

    def get_nearest(self, sampled_point):
        if not self.nodes:
            return None
        self.build_kd_tree()
        distance, index = self.kd_tree.query(sampled_point)
        return self.nodes[index]

    def steer(self, from_node, to_point, step_size):
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        dist = np.hypot(dx, dy)
        if dist <= step_size:
            new_x, new_y = to_point
        else:
            theta = np.arctan2(dy, dx)
            new_x = from_node.x + step_size * np.cos(theta)
            new_y = from_node.y + step_size * np.sin(theta)
        return Node(int(new_x), int(new_y), parent=from_node)

def load_map(map_image_path):
    map_img = cv2.imread(map_image_path, 0)  # Load in grayscale
    return cv2.threshold(map_img, 127, 255, cv2.THRESH_BINARY)[1]  # Binary threshold

def sample_point(map_shape):
    point = random.randint(0, map_shape[1] - 1), random.randint(0, map_shape[0] - 1)
    goal_point = (100, 100)
    x = np.random.choice([1, 2], p=[0.4, 0.6]) 
    return point if x == 1 else goal_point

def is_free(map_img, point, clearance=40):
    x, y = point
    patch = map_img[max(0, y-clearance):min(map_img.shape[0], y+clearance+1), 
                    max(0, x-clearance):min(map_img.shape[1], x+clearance+1)]
    return np.all(patch == 255)

def distance(node1, node2):
    return np.hypot(node1.x - node2.x, node1.y - node2.y)

def rrt_star(map_img, start, goal, max_iterations, step_size=20, clearance=40):
    rrt = RRTStar()
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    rrt.add_node(start_node)

    plt.figure()
    plt.imshow(map_img, cmap='gray')
    plt.scatter([start[0], goal[0]], [start[1], goal[1]], color='yellow', s=100)
    plt.title('RRT* Path Planning')

    for i in range(max_iterations):
        x, y = sample_point(map_img.shape)
        if not is_free(map_img, (x, y), clearance):
            continue

        nearest_node = rrt.get_nearest((x, y))
        new_node = rrt.steer(nearest_node, (x, y), step_size)

        if not is_free(map_img, (new_node.x, new_node.y), clearance):
            continue

        new_node.cost = nearest_node.cost + distance(nearest_node, new_node)
        rrt.add_node(new_node)

        plt.plot([nearest_node.x, new_node.x], [nearest_node.y, new_node.y], 'g-', linewidth=0.5)
        plt.plot(new_node.x, new_node.y, 'bo', markersize=2)
        plt.pause(0.01)

        if distance(new_node, goal_node) < step_size:
            goal_node.parent = new_node
            print("Path to goal found.")
            break

    path = []
    node = goal_node
    while node.parent is not None:
        path.append((node.x, node.y))
        node = node.parent
    path.append((start_node.x, start_node.y))
    path.reverse()

    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'b-', linewidth=2)
    plt.show()

    return path, rrt.nodes

def pixel_to_whycon(imgx, imgy):
    goal_x = 0.02537 * imgx - 12.66
    goal_y = 0.02534 * imgy - 12.57
    goal_z = 27.0
    return [goal_x, goal_y, goal_z]

def replan_optimized_path(map_img, initial_path, step_size=70, clearance=40):
    rrt = RRTStar()
    start_node = Node(initial_path[0][0], initial_path[0][1])
    goal_node = Node(initial_path[-1][0], initial_path[-1][1])
    rrt.add_node(start_node)
    path = []
    path.append((start_node.x, start_node.y))
    
    for i in range(1, len(initial_path), 3):
        x, y = initial_path[i]
        path.append((x, y))
    path.append((initial_path[-1][0], initial_path[-1][1]))
    path.reverse()
    

    return path

def plot_path(map_img, initial_path, optimized_path):
    plt.figure()
    plt.imshow(map_img, cmap='gray')
    # Plot initial path
    initial_x, initial_y = zip(*initial_path)
    plt.plot(initial_x, initial_y, 'b-', linewidth=1, label="Initial Path")
    # Plot optimized path
    opt_x, opt_y = zip(*optimized_path)
    plt.plot(opt_x, opt_y, 'r-', linewidth=2, label="Optimized Path")
    plt.scatter([initial_path[0][0], initial_path[-1][0]], [initial_path[0][1], initial_path[-1][1]], 
                color='yellow', s=100, label="Start/Goal")
    plt.legend()
    plt.title('Initial vs Optimized Path')
    plt.show()

# Load map and run initial RRT*
map_img = load_map('/home/nvidia/galileo2024/src/navigation2/scripts/mymap.pgm')
start = (379,141)
goal = (400, 144)
initial_path, _ = rrt_star(map_img, start, goal, max_iterations=10000)
print("Initial path length:", len(initial_path))

# Re-plan for optimized path
optimized_path = replan_optimized_path(map_img, initial_path)
print("Optimized path length:", len(optimized_path))

# Convert to WhyCon format
final_path = [pixel_to_whycon(x, y) for x, y in optimized_path]
print("Final optimized path in WhyCon format:", final_path)

# Plot initial and optimized paths
plot_path(map_img, initial_path, optimized_path)
