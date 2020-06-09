from map_visualizer import MapVisualizer
from world import World
from rrt_star import RrtStar
from shapely.geometry import Polygon
import numpy as np

global time
time = 0.0

def compute_pose(pose, dt):
    global time
    time = time + dt
    pose = np.array([np.cos(time), np.sin(2*time), np.pi*((time//5)%2)])
    return pose

def move_func_custom(robot_pose, dt):
    pose = robot_pose
    pose += dt
    print(pose)
    return pose

polygon_array = np.array([Polygon([(-3,-3), (-3,-2), (-2,-2), (-2,-3)]),
                    Polygon([(2,2), (2,3), (3,3), (3,2)])])
robot_polygon = Polygon([(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (0, 0.75), (-0.5, 0.5)])
robot_pose = np.array([0,0,0])
world = World(polygon_array, robot_polygon, robot_pose)
goal = np.array([1,1,0])
eta = 0.2
N = 10000
rrt = RrtStar(eta, world)
global path, index
path = rrt.search(robot_pose, goal, N=N)
print(path)
index = 0

def move_on_path(pose, dt):
    global path, index
    if index < len(path)-1:
        index += 1
        return path[index]
    else:
        return pose

map_viz = MapVisualizer(world, 1000, move_on_path)
map_viz.open_window()