import cv2
import numpy as np
import shapely
from shapely.geometry import Point, Polygon, LineString
from world import world
import random as rd


class RRT_STAR():
    def __init__(self, world, radius, n, neighbor_radius, max_edge_length):
        self.world = world
        self.goal = None
        self.radius = radius
        self.iterations_limit = n
        self.parents = dict()
        self.costs = dict()
        self.vertices = []
        self.edges = []
        self.ready = False
        self.neighbor_radius = neighbor_radius
        self.max_edge_length = max_edge_length
        self.path = []

    def distance(self, pose1, pose2):
        d = np.sqrt((pose1[0]-pose2[0])**2 + (pose1[1]-pose2[1])**2)
        return d
    
    def nearest(self, pose):
        #For this function to work properly, we need to make sure that the graph is not void of any points.
        d = np.inf
        vertex = None
        for i in range(len(self.vertices)):
            d1 = self.distance(self.vertices[i], pose)
            if d1 < d:
                d = d1
                vertex = self.vertices[i]
        return vertex, d

    def find_neighbors(self, pose, radius):
        #returns the indexes of the neighbors from the vertices list.
        neighbors = []
        for i in range(len(self.vertices)):
            d = self.distance(self.vertices[i], pose)
            if d <= radius:
                neighbors.append(i)
        return neighbors

    def initialize_search(self, goal, epsilon, radius, iterations):
        #goal: pose  ___  epsilon, radius: float;  ___   iterations: int
        self.goal = goal
        self.parents[0] = None
        self.vertices.append(self.world.pose)
        self.costs[0]=0
        self.ready = True
    
    
    def is_safe(self, pose0, pose1):
        print(self.vertices)
        robot = self.world.robot.copy()
        robot0 = self.world.simulate_pose(pose0)
        robot1 = self.world.simulate_pose(pose1)
        cenroid0 = self.world.get_centroid(self.world.simulate_pose(pose0))
        centroid1 = self.world.get_centroid(self.world.simulate_pose(pose1))
        #will this really do it though ?
        link = LineString([tuple(pose0), tuple(pose1)])
        print("show me", link)
        safe_passage = True
        for obstacle in self.world.obstacles:
            print(obstacle)
            polygon_obstacle = Polygon([tuple(obstacle[0][i]) for i in range(len(obstacle[0]))])
            if link.intersects(polygon_obstacle):
                safe_passage = False
                return safe_passage
        return safe_passage

    def search_edges(self, pose):
        #returns list of edges where the pose is involved
        edges = []
        for edge in self.edges:
            if pose in edge:
                edges.append(edge)
        return edges

    def great_grandparent(self, child_index, parent_index):
        current_index = child_index
        while not current_index is None:
            current_index = self.parents[current_index]
            if current_index == parent_index:
                print('returning true')
                return True
        return False

    def expand_edge(self):
        pose = rd.choice(self.vertices)

        x = int(self.max_edge_length*(rd.random()-1/2))
        y = int(np.sqrt(self.max_edge_length**2-x**2))
        new_pose = [pose[0] + x, pose[1] + y, 2*np.pi*rd.random()]
        parent, d = self.nearest(new_pose)
        
        return new_pose
        


    def search(self):
        if self.ready:
            i = 0
            while i < self.iterations_limit:
                new_pose = self.expand_edge()
                parent_pose, distance_to_parent = self.nearest(new_pose)
                #new_pose = self.limit_edge(new_pose)
                if not self.world.possible_pose(new_pose):
                    print("no")
                    continue
                print("yes")
                
                self.vertices.append(new_pose)
                parent_index = self.vertices.index(parent_pose)
                self.costs[i+1] = self.costs[parent_index] + distance_to_parent
                self.parents[i+1] = parent_index
                self.edges.append([parent_pose, new_pose])
                #Until here, everything seems to be fine, although edges are not really used, they'll be deleted afterwards.
                
                
                cv2.line(self.world.canvas, tuple(parent_pose[:2]), tuple(new_pose[:2]), (0,0,0))
                #Before we go ahead, we look if any of the closer vertices can get a lower by switching their parent to the new node.
                neighbors_list = self.find_neighbors(new_pose, self.neighbor_radius)
                for j in neighbors_list:
                    if self.costs[i+1] + self.distance(new_pose, self.vertices[j]) < self.costs[j] and self.vertices[j] != new_pose : #and no loop of parents here:
                        print('an update is happening: ', i)
                        #print(self.costs[j])
                        self.costs[j] = self.costs[i+1] + self.distance(new_pose, self.vertices[j])
                        print(self.costs[i], self.distance(new_pose, self.vertices[j]))
                        #update the edge
                        #translated = tuple(self.vertices[j][:2])[0]+3,
                        cv2.line(self.world.canvas, tuple(self.vertices[j][:2]), tuple(self.vertices[self.parents[j]][:2]), (.9, .9, .9))
                        #set new element as parent of j, but the parent of enw element is j...
                        self.parents[j] = i+1
                        cv2.line(self.world.canvas, tuple(self.vertices[j][:2]), tuple(new_pose[:2]), (0,0,0))
                i+=1
                self.world.show_image()
            """for i in range(len(self.vertices)):
                if not self.parents[i] is None:
                    cv2.line(self.world.canvas, tuple(self.vertices[i][:2]), tuple(self.vertices[self.parents[i]][:2]), (0,0,0))
            self.world.show_image()"""

    def get_path(self, goal, error_radius):
        #path is a list of consecutive poses generated for the robot
        self.path.append(goal) #adding goal to path 
        final_vertex = self.nearest(goal)[0]
        current_index = self.vertices.index(final_vertex) 
        self.path.append(final_vertex) #adding closest parent of the goal to the path
        cv2.line(self.world.canvas, tuple(final_vertex[:2]), tuple(goal[:2]), (.5,.5,0), thickness = 3)
        self.world.show_image()
        while not self.parents[current_index] is None:
            parent_index = self.parents[current_index] #updating the parent
            cv2.line(self.world.canvas, tuple(self.vertices[current_index][:2]), tuple(self.vertices[parent_index][:2]), (.5,.5,0), thickness = 3)
            current_index = parent_index
            self.path.append(self.vertices[current_index]) #adding the new parent to the list
            self.world.show_image()
        self.path.append(self.world.pose) #adding the initial pose
        self.path.reverse()
        return self.path
    
    def follow_path(self):
        #function workscorrectly so the translate function is the problem
        if len(self.path) != 0:
            for i in range(1, len(self.path)):
                self.world.rotate_robot(self.path[i][2])
                self.world.translate_robot(self.path[i])
                self.world.show_image()
                cv2.waitKey(100)
            """for i in range(len(self.path)):
                cv2.circle(self.world.canvas, (int(self.path[i][0]), int(self.path[i][1])), 4, (0, 0, 0), -1)
                self.world.show_image()"""
                







    """def search(self, goal, radius, n):
        i = 0
        while i <= n:
            x_pot, y_pot, theta_pot = radius*rd.random(), radius*rd.random(), 2*np.pi*rd.random()
            pose_pot = [x_pot, y_pot, theta_pot]
            retry = False
            for obstacle in self.obstacles:
                if self.intersect(pose_pot, obstacle):
                    retry == True
                    break
            #Need to define the nearest function
            pose_nearest, distance_nearest = self.nearest(pose_pot)
            self.cost[pose_pot] = distance_nearest
            #Needs a find neighbors function
            pose_best, pose_neighbors = self.find_neighbors(pose_pot, radius, n)
            link = [pose_new, pose_best]
            for pose_index in pose_neighbors:
                pose = self.vertices[pose_index]
                if self.cost[pose_pot] + self.distance(pose_pot, pose) < self.cost[pose]:
                    self.cost[pose] = self.cost[pose_pot] + self.distance(pose_pot, pose)
                    self.parent[pose] = pose_pot
                    self.vertices += pose_pot, pose
            self.edges.append(link)
        print(self.vertices)"""




    #triangle = np.array([[[x1, y1], [x2, y2], [x3, y3]]], np.int32)
my_canvas = np.ones((1920, 1920, 3))*.9
my_robot = np.array([[[40, 40], [40, 100], [60, 100], [60, 60],[100, 60], [100, 100], [120, 100], [120, 40]]], np.int32)
my_obstacles = [np.array([[[240, 130], [380, 230], [10, 280]]], np.int32), np.array([[[500, 130], [720, 109], [620, 100], [640, 125]]], np.int32),np.array([[[10, 800], [10, 910], [1700, 910], [1700, 800]]], np.int32) ]
my_pose = [512, 512, 0]
my_world = world(my_canvas, my_robot, my_obstacles, my_pose)

my_rrt = RRT_STAR(my_world, 1920, 1000, 100, 100)
my_rrt.initialize_search([900, 1860, 0], 20, 300, 100)
my_rrt.search()
my_rrt.get_path([900, 1860, 0], 1000)
my_rrt.follow_path()
cv2.waitKey()

#my_world.show_image()
"""n = 200
for i in range(n):
    
    print(my_world.intersect(my_world.pose, my_world.obstacles[0]))
    #my_world.translate_robot((2,2,1))
    my_world.rotate_robot(.2*np.pi)
    my_world.show_image()"""