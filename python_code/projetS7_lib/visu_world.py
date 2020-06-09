from world import World
from rrt_star import RrtStar
import numpy as np
import cv2
from shapely.geometry import Polygon, LinearRing
import os
import shutil

class VisuWorld():

    COLOR_OBSTACLES = (0,0,0)
    COLOR_ROBOT = (0,0,255)
    """CLASS_DIR = os.path.dirname(os.path.realpath(__file__))
    IMAGES_DIR = CLASS_DIR + '/world_visualizations/'"""

    def __init__(self, img_width, img_height, IMAGES_DIR):
        """
        Initialize visualization

        Args:
            img_width: width of OpenCV canvas (pixels)
            img_height: height of OpenCV canvas (pixels)
        """
        self.width = img_width
        self.height = img_height
        self.IMAGES_DIR = IMAGES_DIR

    def draw(self, world, robot_pose, filename, base_canvas=None):
        """
        Draws the world with the robot at robot_pose and saves it in a jpeg file filename.

        Args:
            world: world to draw
            robot_pose: pose at which to draw the robot
            filename: name of jpeg file
            base_canvas: canvas to reuse, reinitialized if equal to None

        Returns:
            canvas: the image matrix
        """
        if base_canvas is None:
            canvas = np.ones((self.height, self.width, 3))*255
        else:
            canvas = base_canvas
        
        bounding_box = world.get_bounding_box()
        obstacles = [self._get_polygon_opencv_coords(obs, bounding_box) for obs in world.obstacles]
        cv2.fillPoly(canvas, obstacles, self.COLOR_OBSTACLES)
        robot = self._get_polygon_opencv_coords(world.get_robot(robot_pose), bounding_box)
        cv2.fillPoly(canvas, [robot], self.COLOR_ROBOT)
        cv2.imwrite(filename, canvas)

        return canvas

    def draw_frames(self, world, pose_list, filename_template):
        """
        Draws the world with the robot moving as described by pose_list,
        and saves the images in jpeg files filename_template00x.jpeg
        
        Args:
            world: world to draw
            pose_list: list of poses at which to draw the robot
            filename_template: template used to write images on files
        """
        directory = self.IMAGES_DIR + filename_template
        if os.path.isdir(directory):
            shutil.rmtree(directory)
        os.makedirs(directory)
        
        num = 1
        for pose in pose_list:
            self.draw(
                world,
                pose,
                directory + '/' + filename_template + "_{:06d}.jpg".format(num)
            )
            num += 1

    def multidraw(self, world, pose_list, filename_template):
        """
        Draws the world with the robot moving as described by pose_list,
        and saves the images in jpeg files filename_template00x.jpeg
        
        The robot is erased smoothly from one image to the next one
        """
        directory = self.IMAGES_DIR + filename_template
        if os.path.isdir(directory):
            shutil.rmtree(directory)
        os.makedirs(directory)
        
        num = 1
        canvas = np.ones((self.height, self.width, 3))*255
        white = np.ones((self.height, self.width, 3))*255
        for pose in pose_list:
            canvas = self.draw(
                world,
                pose,
                directory + '/' + filename_template + "_{:06d}.jpg".format(num),
                base_canvas=0.99*canvas + 0.01*white
            )
            num += 1

    def _get_polygon_opencv_coords(self, polygon, bounding_box):
        """
        Transform polygon coordinates into OpenCV coordinates
        """
        min_x, min_y, max_x, max_y = bounding_box
        coords_array = np.array(polygon.exterior.coords, dtype=float)
        # coordinates in [0,1]
        norm_coords_array = np.zeros(np.shape(coords_array))
        norm_coords_array[:,0] = (coords_array[:,0] - min_x)/(max_x-min_x)
        norm_coords_array[:,1] = (coords_array[:,1] - min_y)/(max_y-min_y)
        # y-axis inverted
        norm_coords_array[:,1] = 1 - norm_coords_array[:,1]
        # coordinates appropriate for opencv visualization
        norm_coords_array[:,0] = self.width * norm_coords_array[:,0]
        norm_coords_array[:,1] = self.height * norm_coords_array[:,1]
        
        return norm_coords_array.astype(int)
