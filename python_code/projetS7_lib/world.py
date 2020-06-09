import numpy as np
import yaml
from shapely.geometry import Polygon
from shapely import affinity
import os

class World():

    """
    YAML file

    (x,y)min
    (x,y)max
    obstacles
    relative_robot_polygon
    """

    """CLASS_DIR = os.path.dirname(os.path.realpath(__file__))
    WORLDS_DIR = CLASS_DIR + '/worlds/'
    ROBOTS_DIR = CLASS_DIR + '/robots/'"""

    def __init__(self, world_filename, WORLDS_DIR, ROBOTS_DIR):
        """
        Initialize a world from a YAML file describing this world

        Args:
            world_filename: name of the file describing the world (without .yaml) (ie world_tuto)
        """
        self.WORLDS_DIR = WORLDS_DIR
        self.ROBOTS_DIR = ROBOTS_DIR

        # Load dictionary from yaml file describing world
        stream = open(self.WORLDS_DIR + world_filename + ".yaml")
        loaded_dict = yaml.load(stream)
        stream.close()

        # Get bounding box
        min_x = loaded_dict['bound_min']['x']
        min_y = loaded_dict['bound_min']['y']
        max_x = loaded_dict['bound_max']['x']
        max_y = loaded_dict['bound_max']['y']
        self.bounding_box = (min_x, min_y, max_x, max_y)

        # Get robot polygon relative to (0,0)
        robot_name = loaded_dict['robot_name']
        stream = open(self.ROBOTS_DIR + robot_name + ".yaml")
        robot_dict = yaml.load(stream)
        stream.close()
        self.relative_robot_polygon = Polygon(robot_dict['relative_robot_polygon'])

        # Get robot radius (used in distance)
        self.robot_radius = self._get_farthest_dist_from_origin(self.relative_robot_polygon)

        # Get array of obstacle polygons
        self.obstacles = np.array([])
        obstacles_coords_list = loaded_dict['obstacles']
        for obs in obstacles_coords_list:
            self.obstacles = np.append(self.obstacles, Polygon(obs))

    def do_intersect(self, pose):
        """
        Checks if robot polygon at pose intersects with any obstacle or is outside bounding box

        Returns:
            True if robot polygon intersect with obstacles, False otherwise
        """
        moved_robot = self.get_robot(pose)

        # Check if robot is in bounding box
        min_x, min_y, max_x, max_y = self.bounding_box
        for vertex in list(moved_robot.exterior.coords):
            if not (min_x <= vertex[0] <= max_x):
                return True
            if not (min_y <= vertex[1] <= max_y):
                return True
        
        # Check if robot instersects with any obstacle
        for obs in self.obstacles:
            if moved_robot.intersects(obs):
                return True
        
        return False
        

    def free_path(self, pose1, pose2, nb_inter=10):
        """
        Checks feasability of move from pose1 to pose2 for nb_inter intermediate poses

        Returns:
            True if move is possible, False otherwise
        """
        interpolated_path = self.interpolate_path(
            np.array([pose1, pose2]),
            self.dist(pose1, pose2) / nb_inter
        )

        for pose in interpolated_path:
            if self.do_intersect(pose):
                return False
        
        return True

    def interpolate_path(self, path, step):
        """
        Interpolates path to get poses distant by step

        Args:
            path: list of poses to interpolate
            step: (float) max distance from one new pose to the next 

        Returns:
            interpolated list of poses
        """
        if len(path) < 2:
            return path
        
        # Making sure we have floats to avoid problems with integers
        path = path.astype(float)
        
        interpolated_path = np.empty((0,3))
        interpolated_path = np.append(interpolated_path, [path[0]], axis=0)

        for i in range(len(path)-1):
            pose1 = path[i]
            pose2 = path[i+1]

            if abs(pose1[2] - pose2[2]) > np.pi:
                if pose1[2] < np.pi:
                    pose1[2] += 2*np.pi
                else:
                    pose1[2] -= 2*np.pi

            dist = self.dist(pose1, pose2)
            nb_inter = int(np.ceil(dist/step))
            dpose = (pose2 - pose1)/nb_inter
            
            for j in range(1, nb_inter+1):
                pose = pose1 + j*dpose
                pose[2] = pose[2] % (2*np.pi)
                interpolated_path = np.append(interpolated_path, [pose], axis=0)
        
        return interpolated_path


    def get_robot(self, pose):
        """
        Computes robot polygon if it was at pose

        Returns:
        robot_polygon
        """
        # Making sure the pose is correct
        assert len(pose) == 3, "A pose must contain 3 coordinates (x,y,theta)"

        x, y, theta = pose
        robot_polygon = affinity.rotate(self.relative_robot_polygon, theta, origin=(0,0), use_radians=True)
        robot_polygon = affinity.translate(robot_polygon, xoff=x, yoff=y)

        return robot_polygon

    def get_bounding_box(self):
        """
        Returns (xmin, ymin, xmax, ymax)
        """
        return self.bounding_box

    def dist(self, pose1, pose2):
        """
        Computes the distance between pose1 and pose2
        """
        assert isinstance(pose1, np.ndarray) and isinstance(pose2, np.ndarray), "Poses must be numpy arrays"
        assert len(pose1) == len(pose2) == 3, "Poses must contain 3 coordinates (x,y,theta)"

        dpose = np.abs(pose2 - pose1)
        # Compute movement made by rotation of robot's farthest point
        dpose[2] = self.robot_radius * min(dpose[2], 2*np.pi - dpose[2])

        return np.sqrt(np.sum(dpose**2))

    def _get_farthest_dist_from_origin(self, polygon):
        """
        Get the distance of the farthest vertex from (0,0)
        
        Algorithm:
        pol_coords = np.array(polygon.exterior.coords)
        max_dist = 0
        for vertex in pol_coords:
            dist = sqrt(sum(e**2 for e in vertex))
            if dist > max_dist:
                max_dist = dist
        return max_dist
        """
        
        pol_coords = np.array(polygon.exterior.coords)
        return np.sqrt(max(np.sum(e**2) for e in pol_coords))
    
    def sample_pose(self, mode='3d'):
        """
        Sample a random pose (x,y,theta)

        Args:
            mode: '3d' samples pose in 3d, '2d' samples pose in 2d

        Returns:
            pose: (x,y,theta)
        """
        min_x, min_y, max_x, max_y = self.bounding_box

        rand_x = min_x + (max_x - min_x) * np.random.rand()
        rand_y = min_y + (max_y - min_y) * np.random.rand()
        
        if mode == '3d':
            rand_theta = 2*np.pi * np.random.rand()
        else:
            rand_theta = 0

        return np.array([rand_x, rand_y, rand_theta])