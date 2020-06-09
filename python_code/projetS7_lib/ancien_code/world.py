"""
Class managing the space in which the robot moves
x and y coordinates are expressed in meters
theta coordinate is expressed in radians on [0, 2*pi[
"""

from shapely.geometry import Point, Polygon
from shapely import affinity
import numpy as np
import copy
from multiprocessing import RLock

class World:

    def __init__(self, obstacles_array, robot_polygon):
        """
        Initialize the world space 

        Args:
            obstacles_array: array of polygons which compose the map
            robot_polygon: polygon of the robot (relative to position (0,0))
        
        Raise:
            ValueError if the initial pose of the robot intersects with the obstacles
        """
        self._obstacles_array = obstacles_array
        self._obstacles_array_lock = RLock()
        self._relative_robot_polygon = robot_polygon
        
    # USELESS FOR NOW
    def add_polygon(self, polygon):
        """
        Add a polygon to the map

        Args:
            polygon: polygon to add to the map
        
        Raise:
            ValueError if the polygon intersects with the current position of the robot
        """
        assert isinstance(polygon, Polygon), "Input polygon must be of type shapely.geometry.Polygon"
        with self._obstacles_array_lock:
            self._obstacles_array = np.append(self._obstacles_array, [polygon])

    # USELESS FOR NOW
    def remove_polygon(self, point):
        """
        Remove from the map all polygons containing point

        Args:
            point: a point contained in the polygon to remove from the map
        """
        assert isinstance(point, Point), "Input point must be of type shapely.geometry.Point"
        pol_list = []
        with self._obstacles_array_lock:
            for polygon in self._obstacles_array:
                if polygon.contains(point):
                    pol_list.append(polygon)
        
        return np.array(pol_list)

    def is_possible_pose(self, pose):
        """
        Check if this pose is feasible for the robot

        Args:
            pose: pose array [x, y, theta] to look for feasibility
        
        Return:
            boolean describing the feasibility of the pose
        """
        robot_polygon = self.get_robot_polygon(self._relative_robot_polygon, pose)
        obstacles_array = self.get_obstacles_array()

        for polygon in obstacles_array:
            if polygon.intersects(robot_polygon):
                return False
        return True

    def is_possible_move(self, initial_pose, velocity, move_duration):
        """
        Check if the move described by the constant velocity vector
        during time move_duration is feasible

        Args:
            initial_pose: pose of the robot before the move
            velocity: array of velocities v_x (m/s), v_y (m/s), v_theta (rad/s)
            move_duration: duration of the move in seconds
        
        Return:
            boolean describing the feasibility of the move
        """
        end_pose = initial_pose + (velocity*move_duration)
        end_pose[3] = end_pose[3] % (2*np.pi)       # Making sure that theta in end_pose is in [0, 2*np.pi[
        return self.coll_free(initial_pose, end_pose)

    def sample_free(self):
        """
        Sample a pose in free space

        Return:
            random pose in the free space of the map
        """
        minx, miny, maxx, maxy = self.get_limits()
        sample = None

        while sample is None:
            x = minx + (maxx - minx)*np.random.random_sample()
            y = miny + (maxy - miny)*np.random.random_sample()
            theta = 2*np.pi*np.random.random_sample()
            sample = np.array([x, y, theta])

            # Verify if sample point is in free space
            #if not self.is_possible_pose(sample):
            #    sample = None
        
        return sample

    def coll_free(self, initial_pose, end_pose, steps=10):
        """
        Check the feasibility of the linear interpolated move between initial_pose
        and end_pose by checking the feasibility of all the step poses

        Args:
            initial_pose: initial pose array [x, y, theta] of the robot (supposed valid)
            end_pose: ending pose array [x, y, theta] of the robot
            steps: (optional) number of step poses to check on the way
        
        Return:
            boolean describing the absence of collisions with obstacles between initial_pose and end_pose
        """
        dpose = (end_pose - initial_pose) / steps
        pose = initial_pose

        for _ in range(steps):
            pose = pose + dpose
            if not self.is_possible_pose(pose):
                return False
        return True

    def get_robot_polygon(self, relative_robot_polygon, pose):
        """
        Transform relative robot polygon according to the robot pose

        Args:
            relative_robot_polygon: polygon defined relatively around point (0., 0.)
            pose: pose array [x, y, theta] of the robot
        
        Return:
            robot polygon according to the robot pose
        """
        assert isinstance(pose, np.ndarray), "A pose must be an array"
        assert len(pose) == 3, "A pose must contain 3 coordinates"

        x, y, theta = pose
        # rotation of the polygon around the origin
        robot_polygon = affinity.rotate(relative_robot_polygon, theta, use_radians=True, origin=(0., 0.))
        # translation of the rotated polygon
        robot_polygon = affinity.translate(robot_polygon, xoff=x, yoff=y)

        return robot_polygon

    def get_limits(self):
        """
        Get the limits (minx, miny, maxx, maxy) of the 2d square containing the map

        Return:
            tuple (minx, miny, maxx, maxy)
        """
        obstacles_array = self.get_obstacles_array()
        inf = np.inf

        if obstacles_array is None:
            return (-inf, -inf, inf, inf)
        
        minx, miny = inf, inf
        maxx, maxy = -inf, -inf

        for polygon in obstacles_array:
            bounds = polygon.bounds
            minx = min(minx, bounds[0])
            miny = min(miny, bounds[1])
            maxx = max(maxx, bounds[2])
            maxy = max(maxy, bounds[3])
        
        return (minx, miny, maxx, maxy)

    def get_obstacles_array(self):
        """
        Get a copy of the array of polygons
        """
        with self._obstacles_array_lock:
            obstacles_array = np.copy(self._obstacles_array)
        return obstacles_array
    
    def get_configuration_space_volume(self):
        """
        Compute the volume of the configuration space to get a higher bound
        for the volume of the free space
        """
        minx, miny, maxx, maxy = self.get_limits()
        workspace_area = (maxx-minx) * (maxy-miny)
        obstacles_area = sum(pol.area for pol in self.get_obstacles_array())
        return (workspace_area - obstacles_area) * 2*np.pi

if __name__ == "__main__":
    obstacles = [Polygon([(0,0), (1,0), (1,1), (0,1)])]
    obstacles_array = np.array(obstacles)
    robot_polygon = Polygon([(0,0), (0.1,0), (0,0.1)])
    world = World(obstacles_array, robot_polygon)
    for obs in world.get_obstacles_array():
        print(list(obs.exterior.coords))
    print(world.get_limits())
    print(world.is_possible_pose(np.array([1.5,1.5,0])))