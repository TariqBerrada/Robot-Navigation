from world import World
import numpy as np
import yaml
import os

class RrtStar():

    """CLASS_DIR = os.path.dirname(os.path.realpath(__file__))
    TREES_DIR = CLASS_DIR + '/trees/'"""

    def __init__(self, world, eta, TREES_DIR):
        """
        RRT* constructor

        Args:
            world: world in which to do the tree search
            eta: maximum distance of a tree edge
        """

        # Parameters
        self.TREES_DIR = TREES_DIR
        self.world = world
        self.eta = eta

        # Internal state
        self.nodes = np.empty((0,3))
        self.parents = {}
        self.costs = {}
        self.gamma = np.inf
        self.goal = None
        self.initialized = False

    def get_edges(self):
        """Return tree edges as pose doubles"""
        ind_edges = list(self.parents.items())
        edges = np.empty((0,2,3))
        for ind_tuple in ind_edges:
            # Special case of start, do not consider edge
            if ind_tuple == (0, None):
                continue
            edge_k = np.array(
                [self.nodes[ind_tuple[0]], self.nodes[ind_tuple[1]]]
            )
            edges = np.append(edges, [edge_k], axis=0)
        return edges

    def get_path(self):
        """
        returns sorted list of positions describing path from start to goal
        """
        assert self.initialized, "Search has not been initialized yet"

        # abort if goal not found
        ind_nearest_from_goal = self._nearest_index(self.goal)
        x_nearest_from_goal = self.nodes[ind_nearest_from_goal]
        if self.world.dist(x_nearest_from_goal, self.goal) > self._get_r_near():
            return None
        
        # build path by backtracking from goal to start
        else:
            path = [self.goal]
            ind = ind_nearest_from_goal
            while ind is not None:
                path.append(self.nodes[ind])
                ind = self.parents[ind]
            path = np.array(path)
            path = np.flip(path, axis=0)
        
        return path

    def initialize_search(self,start,goal):
        """
        Initializes search
        """
        # internal state
        self.goal = goal
        self._update_gamma()

        # initialize search state
        self.nodes = np.append(self.nodes, [start], axis=0)
        self.parents = {0: None}
        self.costs = {0: 0.}
        self.initialized = True

    def search(self,start,goal):
        """
        Expands tree while path between start and goal doesn't exist

        Returns:
            path
        """
        if not self.initialized:
            self.initialize_search(start, goal)
        path = self.get_path()

        while path is None:
            self.expand_tree(N=50)
            path = self.get_path()
        
        return path
    
    def expand_tree(self,N=1):
        """
        Adds potentially N points to the tree
        """
        assert self.initialized, 'Search not initialized.'

        for _ in range(N):
            x_rand = self.world.sample_pose()
            ind_nearest = self._nearest_index(x_rand)
            x_nearest = self.nodes[ind_nearest]
            x_new = self._steer(x_nearest, x_rand)
            if self.world.free_path(x_nearest, x_new):
                neighbors_ind = self._near(x_new)
                ind_near = np.array([i for i in neighbors_ind if self.world.free_path(self.nodes[i], x_new)])
                cost_min = self.costs[ind_nearest] + self.world.dist(x_nearest, x_new)
                ind_min = ind_nearest
                for ind in ind_near:
                    cost = self.costs[ind] + self.world.dist(self.nodes[ind], x_new)
                    if cost < cost_min:
                        cost_min = cost
                        ind_min = ind
                self.nodes = np.append(self.nodes, [x_new], axis=0)
                ind_new = len(self.nodes)-1
                x_min = self.nodes[ind_min]
                self.parents[ind_new] = ind_min
                self.costs[ind_new] = self.costs[ind_min] + self.world.dist(x_min, x_new)
                
                for ind in ind_near:
                    x = self.nodes[ind]
                    if self.costs[ind_new] + self.world.dist(x_new, x) < self.costs[ind]:
                        self.parents[ind] = ind_new
                        self.costs[ind] = self.costs[ind_new] + self.world.dist(x_new, x)

    
    def save(self, filename):
        """
        Saves edges and path in a YAML file
        """
        dict_to_dump = {
            'edges': self.get_edges().tolist(),
            'path': self.get_path().tolist()
        }
        stream = open(self.TREES_DIR + filename + ".yaml", 'w')
        yaml.dump(dict_to_dump, stream)
        stream.close()

    @staticmethod
    def load(filename, TREES_DIR):
        """
        Loads edges and path from a YAML file
        
        Returns:
            edges, path
        """
        """CLASS_DIR = os.path.dirname(os.path.realpath(__file__))
        TREES_DIR = CLASS_DIR + '/trees/'"""
        stream = open(TREES_DIR + filename + ".yaml", 'r')
        loaded_dict = yaml.load(stream)
        stream.close()

        return (
            np.array(loaded_dict['edges']),
            np.array(loaded_dict['path'])
        )
    
    def _get_r_near(self):
        """Radius of neighbor search."""
        n = len(self.nodes)
        return min(self.eta, np.sqrt(self.gamma/np.pi * np.log(n)/n))
    
    def _update_gamma(self):
        """Update the gamma parameter in neighbor search."""
        
        if self.world is None:
            self.gamma = np.inf
            return
        
        min_x, min_y, max_x, max_y = self.world.get_bounding_box()
        config_space_area = (max_x - min_x)*(max_y - min_y)
        for obs in self.world.obstacles:
            config_space_area -= obs.area
        gamma_L = 8*(4/3)* config_space_area * 2*np.pi
        self.gamma = gamma_L

    def _nearest_index(self, pose):
        """
        Get index of nearest node in the tree.

        Args:
            pose: pose (x,y,theta) to look for nearest neighbor.

        Return:
            nearest pose in the tree.
        """
        assert len(self.nodes)>0, 'No nodes.'
        indexes = [i for i in range(len(self.nodes))]
        closest_index = min(
            indexes,
            key=lambda x: self.world.dist(self.nodes[x], pose)
        )
        return closest_index
    
    def _near(self, pose):
        """
        Find indexes of neighborhood of given pose
        The neighborhood size is governed by self._get_r_near()

        Args:
            pose: the pose around which looking for poses

        Return:
            indexes of all poses in the tree with distance to pose less than r_near
        """
        indexes = [i for i in range(len(self.nodes))]
        r_near = self._get_r_near()
        neighborhood = [ind for ind in indexes if self.world.dist(self.nodes[ind], pose) <= r_near]
        return np.array(neighborhood)
    
    def _steer(self, p1, p2):
        """
        Find point at distance eta from p1 in direction to p2

        Args:
            p1: starting pose
            p2: pose to aim for

        Return:
            pose closest to p2 but not farther away than eta from p1
        """
        assert self.world is not None, 'Need map to steer.'

        dist = self.world.dist(p1, p2)
        
        # if p2 is close enough, we do not need to steer
        if dist <= self.eta:
            return p2
        
        # we need to adjust the theta coordinate of p1
        p1 = p1.astype(float)
        p2 = p2.astype(float)
        theta1 = p1[2]
        theta2 = p2[2]
        if abs(theta1 - theta2) > np.pi:
            if theta2 < np.pi:
                p2[2] += 2*np.pi
            else:
                p2[2] -= 2*np.pi
        
        # compute steered pose
        p = p1 + self.eta*(p2 - p1)/dist
        p[2] = p[2] % (2*np.pi)
        
        return p
    