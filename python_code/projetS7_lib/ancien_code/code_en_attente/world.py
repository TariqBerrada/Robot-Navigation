import cv2
import numpy as np
import shapely
from shapely.geometry import Point, Polygon, LineString
import random as rd
class world():
    # a pose is an (x, y, theta) tuple
    def __init__(self, canvas, robot, obstacles, pose0):
        self.obstacles = obstacles
        self.robot = robot

        self.canvas = canvas
        self.centroid =  [ sum((self.robot[0][i][0] for i in range(len(self.robot[0]))))/len(self.robot[0]), sum((self.robot[0][i][1] for i in range(len(self.robot[0]))))/len(self.robot[0]) ]
        self.pose = self.centroid + [pose0[2]]

    def show_image(self):
        for obstacle in self.obstacles:
            cv2.fillPoly(self.canvas, obstacle, (.6, 0, 0))
        cv2.fillPoly(self.canvas, self.robot, (.53, .57, .62), lineType = 4)
        for point in self.robot[0]:
            cv2.circle(self.canvas, tuple(point), 2, (0, 0, 0), -1)
        cv2.imshow('foo', self.canvas)
        cv2.waitKey(1)
        cv2.fillPoly(self.canvas, self.robot, (.9, .9, .9), lineType = 4)

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def translate_robot(self, new_pose):
        #the theta element of the pose is not taken into account.
        delta_x = new_pose[0] - self.centroid[0]
        delta_y = new_pose[1] - self.centroid[1]
        for i in range(len(self.robot[0])):
            self.robot[0][i][0] += delta_x
            self.robot[0][i][1] += delta_y
        #Update the coordinates of therobot after translating it.
        self.centroid =  [ sum((self.robot[0][i][0] for i in range(len(self.robot[0]))))/len(self.robot[0]), sum((self.robot[0][i][1] for i in range(len(self.robot[0]))))/len(self.robot[0]) ]
        self.pose = [self.centroid[0], self.centroid[1], self.pose[2]]

    def get_centroid(self, robot_pose):
        #argument name is imprecise, it's more robot than robot_pose, run simulate pose on the pose
        return [ sum((robot_pose[0][i][0] for i in range(len(robot_pose[0])))), sum((robot_pose[0][i][1] for i in range(len(robot_pose[0])))) ]

    def rotate_robot(self, theta):
        #self.pose = (self.pose[0], self.pose[1], theta)
        theta0=self.pose[2]
        for i in range(len(self.robot[0])):
            #print("initial", self.robot[0][i], centroid)
            #angle = np.arctan(self.robot[0][i][1]/self.robot[0][i][0])
            x = self.robot[0][i][0]-self.centroid[0]
            y = self.robot[0][i][1]-self.centroid[1]
            self.robot[0][i][0] = x*np.cos(theta-theta0) - y*np.sin(theta-theta0) + self.centroid[0]
            self.robot[0][i][1] = x*np.sin(theta-theta0) + y*np.cos(theta-theta0) + self.centroid[1]
        self.pose = (self.pose[0], self.pose[1], theta)
    
    def intersect(self, pose, obstacle):
        robot = self.robot.copy()
        potential_x = pose[0]
        potential_y = pose[1]
        theta = self.pose[2]
        
        for i in range(len(self.robot[0])):
            x = self.robot[0][i][0]-self.centroid[0]
            y = self.robot[0][i][1]-self.centroid[1]
            robot[0][i][0] = x*np.cos(theta) - y*np.sin(theta) + potential_x
            robot[0][i][1] = x*np.sin(theta) + y*np.cos(theta) + potential_y
        robot = [tuple(robot[0][i]) for i in range(len(robot[0]))]
        robot_polygon = Polygon(robot)
        obstacle = [tuple(obstacle[0][i]) for i in range(len(obstacle[0]))]
        obstacle_polygon = Polygon(obstacle)
        return robot_polygon.intersects(obstacle_polygon)
    
    def simulate_pose(self, pose):
        robot = self.robot.copy()
        centroid = self.centroid
        #Translation component
        delta_x = pose[0] - centroid[0]
        delta_y = pose[1] - centroid[1]
        for i in range(len(robot[0])):
            robot[0][i][0] += delta_x
            robot[0][i][1] += delta_y
        #Rotation component
        centroid = self.pose[:2]
        theta = pose[2]
        for i in range(len(robot[0])):
            #print("initial", self.robot[0][i], centroid)
            #angle = np.arctan(robot[0][i][1]/robot[0][i][0])
            x = robot[0][i][0]-centroid[0]
            y = robot[0][i][1]-centroid[1]
            robot[0][i][0] = x*np.cos(theta) - y*np.sin(theta) + centroid[0]
            robot[0][i][1] = x*np.sin(theta) + y*np.cos(theta) + centroid[1]
        return robot

    def possible_pose(self, pose, dim_x=1900, dim_y=1900):
        robot = self.simulate_pose(pose)
        if not (5<=pose[0]<=dim_x and 5<=pose[1]<=dim_y):
                return False
        for obstacle in self.obstacles:
            if self.intersect(pose, obstacle):
                return False
        return True
        


"""my_canvas = np.ones((1920, 1920, 3))*.5
my_robot = np.array([[[40, 40], [100, 100], [100, 190], [40, 100]]], np.int32)
my_obstacles = [np.array([[[240, 130], [380, 230], [10, 280]]], np.int32), np.array([[[500, 130], [720, 109], [620, 100], [640, 125]]], np.int32)]
my_pose = [512, 512, 0]
my_world = world(my_canvas, my_robot, my_obstacles, my_pose)

n = 200
for i in range(n):
    
    print(my_world.intersect(my_world.pose, my_world.obstacles[0]))
    my_world.translate_robot((1,2,1))
    my_world.rotate_robot(np.pi)
    my_world.show_image()"""