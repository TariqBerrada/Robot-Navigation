#!/usr/bin/env python2

from projetS7_lib.visu_world import VisuWorld
from projetS7_lib.world import World
from projetS7_lib.rrt_star import RrtStar
import numpy as np
import os

CLASS_DIR = os.path.dirname(os.path.realpath(__file__))
RESOURCES_DIR = os.path.dirname(CLASS_DIR) + '/resources'
WORLDS_DIR = RESOURCES_DIR + '/worlds/'
ROBOTS_DIR = RESOURCES_DIR + '/robots/'
TREES_DIR = RESOURCES_DIR + '/trees/'
IMAGES_DIR = RESOURCES_DIR + '/world_visualizations/'

def test_draw():
    world = World('world_tuto', WORLDS_DIR, ROBOTS_DIR)
    visu_world = VisuWorld(500, 500, IMAGES_DIR)
    visu_world.draw(
        world,
        np.array([-3,2,np.pi/4]),
        "test.jpg"
    )

def test_opencv_coords():
    world = World('world_tuto', WORLDS_DIR, ROBOTS_DIR)
    visu_world = VisuWorld(500, 500, IMAGES_DIR)
    bounding_box = world.get_bounding_box()
    print("Bounding box :", bounding_box)
    new_coords = visu_world._get_polygon_opencv_coords(world.obstacles[0], bounding_box)
    print("OpenCV coords :", new_coords)

def test_draw_frames():
    world = World('world_tuto', WORLDS_DIR, ROBOTS_DIR)
    visu_world = VisuWorld(500, 200, IMAGES_DIR)
    rrt = RrtStar(world, 0.5, TREES_DIR)
    start = np.array([-3, -3, 0])
    goal = np.array([4,-2,np.pi/4])
    path = rrt.search(start, goal)
    rrt.save('tree_tuto')
    interpolated_path = world.interpolate_path(path, 0.01)
    visu_world.draw_frames(world, interpolated_path, 'world_tuto')

def test_multidraw():
    world = World('world_tuto', WORLDS_DIR, ROBOTS_DIR)
    visu_world = VisuWorld(200, 500, IMAGES_DIR)
    rrt = RrtStar(world, 0.5, TREES_DIR)
    start = np.array([-3, -3, 0])
    goal = np.array([4,-2,np.pi/4])
    path = rrt.search(start, goal)
    rrt.save('tree_tuto')
    interpolated_path = world.interpolate_path(path, 0.01)
    visu_world.multidraw(world, interpolated_path, 'world_tuto_fade')

if __name__ == "__main__":
    test_draw()
    test_opencv_coords()
    test_draw_frames()
    test_multidraw()