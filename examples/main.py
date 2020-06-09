from projetS7_lib.rrt_star import RrtStar
from projetS7_lib.world import World
from projetS7_lib.visu_world import VisuWorld
from projetS7_lib.visu_3d import Visu3D
import numpy as np
import os

CLASS_DIR = os.path.dirname(os.path.realpath(__file__))
RESOURCES_DIR = os.path.dirname(CLASS_DIR) + '/resources'
WORLDS_DIR = RESOURCES_DIR + '/worlds/'
ROBOTS_DIR = RESOURCES_DIR + '/robots/'
TREES_DIR = RESOURCES_DIR + '/trees/'
IMAGES_DIR = RESOURCES_DIR + '/world_visualizations/'

world_file="world_1"
world=World(world_file, WORLDS_DIR, ROBOTS_DIR)
eta=0.5
rrt=RrtStar(world,eta, TREES_DIR)
start = np.array([-3, -3, 0])
goal = np.array([3, 3, np.pi])
path = rrt.search(start, goal)
rrt.save("tree_world1")

edges, path = RrtStar.load("tree_world1", TREES_DIR)
width = 1000
height = 800
visu_world = VisuWorld(width, height, IMAGES_DIR)
interpolated_path = world.interpolate_path(path, step=0.05)
visu_2d=visu_world.draw_frames(world, interpolated_path,'world1')

mode = 'both'
obstacles_cloud_filename = "obstacle_world_1"
visu_3d = Visu3D(
    "world_1",
    "tree_world1",
    WORLDS_DIR,
    TREES_DIR,
    OBSTACLES_DIR,
    ROBOTS_DIR,
    mode=mode,
    obstacles_cloud_filename=obstacles_cloud_filename
)
