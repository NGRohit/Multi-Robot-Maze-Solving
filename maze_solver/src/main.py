#!/usr/bin/env python
# license removed for brevity
import subprocess
import rospy

import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import yaml
import logging
import threading

from maze import Maze

from astar_search import astar_search
from maze_problem import MazeProblem

config = {
    "map_dir": "map",
    "map_info":"test.yaml",
    "algorithm":"astar"
}

def main():

    os.chdir(r'./src/maze_solver')

    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

    # open map yaml file
    with open(os.path.join(config["map_dir"], config["map_info"])) as file:
        map_config = yaml.safe_load(file)

    # Read image
    input = cv2.imread(os.path.join(config["map_dir"], map_config["image"]), -1)

    # Make wall = 1, path = 0
    input = (input != 254).astype(int)
    
    print(list(loc_to_index(9.5, 8)))
    test_maze1 = Maze(input, list(loc_to_index(9.3, 8)))
    # test_mp = MazeProblem(test_maze3, loc_to_index(8, 8))
    test_mp1 = MazeProblem(test_maze1, loc_to_index(-9.5, -7.5))

    # this should explore a lot of nodes; it's just uniform-cost search
    result1 = astar_search(test_mp1, test_mp1.manhattan_heuristic)
    print(result1)
    # result1.move('robot_0')

    test_maze2 = Maze(input, list(loc_to_index(-5, 5)))
    # test_mp = MazeProblem(test_maze3, loc_to_index(8, 8))
    test_mp2 = MazeProblem(test_maze2, loc_to_index(9.3, -8))

    # this should explore a lot of nodes; it's just uniform-cost search
    result2 = astar_search(test_mp2, test_mp2.manhattan_heuristic)
    print(result2)
    # result2.move('robot_1')

    x = threading.Thread(target=result1.move, args=('robot_0',))
    x.start()

    x1 = threading.Thread(target=result2.move, args=('robot_1',))
    x1.start()
    
    # Maze input
    # plt.imshow(input, cmap='gray')
    # plt.title("Maze Map")
    # plt.axis("off")
    # plt.show() 

def loc_to_index(x, y):
    return int(x * 10 + 100), int(100 - y * 10)

def index_to_loc(x, y):
    return float(x - 100) / 10, float(100 - y) / 10


if __name__ == '__main__':
    main()
