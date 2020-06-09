from st5generic.grid_map import GridMap
from st5_generic.scan_utils import ScanBuilder
from st5_generic.geometry  import *
from world import World
from shapely.geometry import Polygon,Point
import rospy
import tf2_ros
from geometry_msgs.msg import (TransformStamped, PoseStamped, PoseArray,
                               Transform, Quaternion,
                               PoseWithCovarianceStamped)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from amcl import 
class Node001():
    def __init__(self,inflate_radius):
        """
        args: inflate_radius:min safety distance to obstacles
        """
        self.inflate_radius=inflate_radius

        #publishers
        self.path_pub = rospy.Publisher('/plan', Path, queue_size=1)

        #subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb,
                                        queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb,
                                         queue_size=1)
        self.initialpose_sub = rospy.Subscriber(
            '/initialpose', PoseWithCovarianceStamped, self.initialpose_cb,
            queue_size=1)
    def map_of(self, world):
        """
        calculates Occupancy grip from a world

        args:
        world: World

        returns:
        OccupancyGrid
        """
        resolution=0.05
        
        (x_min,x_max,y_min,y_max))=world.get_bounding_box()
        init_size=resolution*max(x_max-x_min,y_max-y_min)
        map_grid=GridMap(resolution,init_size,growth=init_size)
        for i in range(init_size+1):
            for j in range(init_size+1):
                x,y=map_grip.index_to_coords(i,j)
                point=Point(x,y)
                for obs in world.obstacles:
                    if point.intersects(obs):
                        map_grid.set_cell(i,j,1)
                    else:
                        map.set_cell(i,j,0)
        inflated_map_grid=map_grid.inflate(self.inflate_radius)
        return inflated_map_grid

    def global_planner(self,path):
        """
        Sends path as the global plan

        args:
        path: list of poses
        """
        

    def local_planner(self, move_base):
        """
        computes local planner
        """

        pass

    def localization(self):
        """
        computes and returns the robot's pose

        """
        pass

    