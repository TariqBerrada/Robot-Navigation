import numpy as np
from world import World
from shapely.geometry import Polygon
import cv2
from rrt_star import RrtStar

class RRTVisualizer:

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

        self.rrt = RrtStar(0.1, self.world)
        self.rrt.initialize_search(start, goal)
    
    def get_opencv_coords(self, pose):
        """
        Transform polygon coordinates into OpenCV coordinates
        """
        pose_2d = pose[:2]
        # coordinates in [0,1]
        norm_coords_array = (pose_2d - self.min_bound)/(self.max_bound-self.min_bound)
        # y-axis inverted
        norm_coords_array[1] = 1 - norm_coords_array[1]
        # coordinates appropriate for opencv visualization
        x, y = (self.resolution*norm_coords_array).astype(int)
        return (x, y)

    def draw_tree(self):
        """
        Draw map and make robot move as specified by RRT* path
        For now, push Escape key to go to next image
        """
        while True:
            canvas = np.ones((self.resolution, self.resolution, 3))
            self.rrt.expand_tree(50)
            edges = self.rrt.edges
            for edge in edges:
                p1 = self.get_opencv_coords(edge[0])
                p2 = self.get_opencv_coords(edge[1])
                cv2.line(canvas, p1, p2, (0,0,0), thickness=1)
            path = self.rrt.current_path
            if path is not None:
                for i in range(len(path)-1):
                    p1 = self.get_opencv_coords(path[i])
                    p2 = self.get_opencv_coords(path[i+1])
                    cv2.line(canvas, p1, p2, (1,0,0), thickness=4)
            cv2.imshow('tree', canvas)
            cv2.waitKey(0)
        cv2.destroyAllWindows()
    

if __name__ == "__main__":
    obstacles_array = np.array([Polygon([(-3,-3), (-3,-2), (-2,-2), (-2,-3)]),
                    Polygon([(2,2), (2,3), (3,3), (3,2)])])
    robot_polygon = Polygon([(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (0, 0.75), (-0.5, 0.5)])
    world = World(obstacles_array, robot_polygon)
    # start and stop to None for test
    start = np.array([-3,-1,0])
    goal = np.array([0,0,0])
    rrt_viz = RRTVisualizer(world, 1000, start, goal)
    rrt_viz.draw_tree()