import numpy as np
from world import World
from shapely.geometry import Polygon
import cv2

class MapVisualizer:

    def __init__(self, world, resolution, start, goal):
        """
        Initialize the map visualizer

        Args:
            world: the description of the map in which the robot evolves
            resolution: resolution of the visualizer
            start: starting pose [x,y,theta] for the RRT* search
            goal: goal pose [x,y,theta] for the RRT* search
        """

        self.world = world
        self.relative_robot_polygon = self.world._relative_robot_polygon
        self.robot_pose = start
        self.resolution = resolution
        self.obstacles_array = self.world.get_obstacles_array()
        limits = self.world.get_limits()
        self.min_bound = min(limits)
        self.max_bound = max(limits)

        # TODO: Search path with RRT*
        self.path = np.array([np.array([k,k,k], dtype=float)/100 for k in range(100)])

    def polygon_normal_coords_to_array(self, polygon):
        """
        Transform polygon coordinates into OpenCV coordinates
        """
        coords_array = np.array(list(polygon.exterior.coords), dtype=float)
        # coordinates in [0,1]
        norm_coords_array = (coords_array - self.min_bound)/(self.max_bound-self.min_bound)
        # y-axis inverted
        norm_coords_array[:,1] = 1 - norm_coords_array[:,1]
        # coordinates appropriate for opencv visualization
        return (self.resolution*norm_coords_array).astype(int)

    def draw_map(self):
        """
        Draw map and make robot move as specified by RRT* path
        For now, push Escape key to go to next image
        """
        #canvas = np.ones((self.resolution, self.resolution, 3))
        obstacles = np.array([self.polygon_normal_coords_to_array(obs) for obs in self.obstacles_array])
        #cv2.fillPoly(canvas, obstacles, (0,0,0))
        for pose in self.path:
            canvas = np.ones((self.resolution, self.resolution, 3))
            cv2.fillPoly(canvas, obstacles, (0,0,0))
            cv2.fillPoly(canvas, obstacles, (0,0,0))
            robot_polygon = self.world.get_robot_polygon(self.relative_robot_polygon, pose)
            robot = self.polygon_normal_coords_to_array(robot_polygon)
            cv2.fillPoly(canvas, [robot], (0,0,1))
            cv2.imshow('map', canvas)
            cv2.waitKey(0)
        cv2.destroyAllWindows()
    

if __name__ == "__main__":
    obstacles_array = np.array([Polygon([(-3,-3), (-3,-2), (-2,-2), (-2,-3)]),
                    Polygon([(2,2), (2,3), (3,3), (3,2)])])
    robot_polygon = Polygon([(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (0, 0.75), (-0.5, 0.5)])
    world = World(obstacles_array, robot_polygon)
    # start and stop to None for test
    map_viz = MapVisualizer(world, 1000, None, None)
    map_viz.draw_map()