from rrt_star import RrtStar
from world import World
import numpy as np
from visu_world import VisuWorld
from visu_3d import Visu3D

world_file="world_1"
world=World(world_file)
eta=0.5
rrt=RrtStar(world,eta)
start = np.array([-3, -3, 0])
goal = np.array([3, 3, np.pi])
path = rrt.search(start, goal)
rrt.save("tree_world1")

edges, path = RrtStar.load("tree_world1")
width = 1000
height = 800
visu_world = VisuWorld(width, height)
interpolated_path = world.interpolate_path(path, step=0.05)
visu_2d=visu_world.draw_frames(world, interpolated_path,'world1')

mode = 'both'
obstacles_cloud_filename = "obstacle_world_1"
visu_3d = Visu3D(
    "world_1",
    "tree_world1",
    mode=mode,
    obstacles_cloud_filename=obstacles_cloud_filename
)
