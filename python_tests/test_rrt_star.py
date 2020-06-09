#!/usr/bin/env python2

from projetS7_lib.rrt_star import RrtStar
from projetS7_lib.world import World
import numpy as np
import os

CLASS_DIR = os.path.dirname(os.path.realpath(__file__))
RESOURCES_DIR = os.path.dirname(CLASS_DIR) + '/resources'
WORLDS_DIR = RESOURCES_DIR + '/worlds/'
ROBOTS_DIR = RESOURCES_DIR + '/robots/'
TREES_DIR = RESOURCES_DIR + '/trees/'

def test_search():
    world = World('world_tuto', WORLDS_DIR, ROBOTS_DIR)
    rrt = RrtStar(world, 1, TREES_DIR)
    start = np.array([-2, -2, 0])
    goal = np.array([3.5, -3, np.pi/4])
    path = rrt.search(start, goal)
    print(path)
    print("-----")
    edges = rrt.get_edges()
    print(edges)
    max_edge_dist = 0
    for edge in edges:
        dist = world.dist(edge[0], edge[1])
        if dist > max_edge_dist:
            max_edge_dist = dist
    print("Maximum edge distance :", max_edge_dist)

def test_steer():
    world = World('world_tuto', WORLDS_DIR, ROBOTS_DIR)
    rrt = RrtStar(world, 0.1, TREES_DIR)
    pos1 = np.array([0,0,0])
    pos2 = np.array([1,0,0])
    pos_steer = rrt._steer(pos1, pos2)
    print("Steered position :", pos_steer)

def test_save_load():
    world = World('world_tuto', WORLDS_DIR, ROBOTS_DIR)
    rrt = RrtStar(world, 0.5, TREES_DIR)
    start = np.array([-2, -2, 0])
    goal = np.array([3.5, -3, np.pi/4])
    path = rrt.search(start, goal)
    edges = rrt.get_edges()

    rrt.save('tree_test')

    load_edges, load_path = RrtStar.load('tree_test', TREES_DIR)
    print("Save/Load working for edges :", np.array_equal(edges, load_edges))
    print("Save/Load working for path :", np.array_equal(path, load_path))

if __name__ == "__main__":
    test_search()
    print("------------------------------")
    test_steer()
    print("------------------------------")
    test_save_load()