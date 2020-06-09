import yaml
import numpy as np
from shapely.geometry import Polygon
from world import World
import os

CLASS_DIR = os.path.dirname(os.path.realpath(__file__))

def test_yaml_load():
    fibo_list = [1,1,2,3,5,8,13,21]
    fibo_str = ""
    for e in fibo_list:
        fibo_str += "- " + str(e) + "\n"
    print("Test load liste python :", fibo_list == yaml.load(fibo_str))
    
    test_dict = {'cle1': fibo_list,
                'cle_compliquee': {'prem': [e**2 for e in range(5)], 'deuz': 'test de string'}}
    print("\n\nFormat YAML du dictionnaire :")
    print(yaml.dump(test_dict))
    print(yaml.load(yaml.dump(test_dict)))
    print("\n\nTest format dictionnaire :", test_dict == yaml.load(yaml.dump(test_dict)))

    test_file = open(CLASS_DIR + '/test_yaml.yaml', 'w')
    yaml.dump(test_dict, test_file, default_flow_style=False)
    test_file.close()

    test_file = open(CLASS_DIR + '/test_yaml2.yaml', 'r')
    load_dict = yaml.load(test_file)
    test_file.close()
    print("\n\nTest dictionnaire charge :")
    print(load_dict)
    print(load_dict == test_dict)

def test_world_tuto():
    world_tuto = {'bound_min': {'x': -5, 'y': -5},
                'bound_max': {'x': 5, 'y': 5},
                'relative_robot_polygon': [
                    [-0.5, -0.5],
                    [0.5, -0.5],
                    [0.5, 0.5],
                    [0, 0.75],
                    [-0.5, 0.5]
                ],
                'obstacles': [
                    [
                        [-5, -0.5],
                        [5, -0.5],
                        [5, 0.5],
                        [-5, 0.5]
                    ],
                    [
                        [3, 3],
                        [4, 3],
                        [4, 4],
                        [3, 4]
                    ]
                ]}
    stream = open(CLASS_DIR + '/worlds/world_tuto.yaml', 'r')
    load_dict = yaml.load(stream)
    stream.close()
    print("\n\nTest if world_tuto.yaml is written in the good way :")
    print(world_tuto == load_dict)
    print("\n")
    print(load_dict)

    print("\n")
    relative_robot_polygon = Polygon(load_dict['relative_robot_polygon'])
    print(list(relative_robot_polygon.exterior.coords))
    world = World('world_tuto')
    print("Distance maximale du robot a l'origine :", world.robot_radius)

def test_intersect():
    world = World('world_tuto')
    should_intersect = world.do_intersect(np.array([-6, -6, 0]))
    should_not_intersect = world.do_intersect(np.array([0, -3, np.pi/2]))
    print("Should be True", should_intersect)
    print("Should be False", should_not_intersect)

def test_interpolate_path():
    world = World('world_tuto')
    path = np.array(
        [
            [0,0,0],
            [1,0,np.pi]
        ]
    )
    step = 0.1
    interpolated_path = world.interpolate_path(path, step)
    print(interpolated_path)
    problems = False
    for i in range(len(interpolated_path)-1):
        if world.dist(interpolated_path[i], interpolated_path[i+1]) > step:
            problems = True
    if problems:
        print("\n[Problem] Interpolation with distance too long")
    else:
        print("\n[All OK] No problem with the distance of interpolation")
    
def test_free_path():
    world = World('world_tuto')
    
    pose0 = np.array([0,-3,np.pi/2])
    pose1 = np.array([3,-2,0])
    should_free = world.free_path(pose0, pose1)
    print("Should be True", should_free)

    pose2 = np.array([0,3,0])
    should_not_free = world.free_path(pose0, pose2, nb_inter=50)
    print("Should be False", should_not_free)

if __name__ == "__main__":
    test_yaml_load()
    print('--------------------')
    test_world_tuto()
    print('--------------------')
    test_intersect()
    print('--------------------')
    test_interpolate_path()
    print('--------------------')
    test_free_path()