"""
RRT* search
"""

from world import World
import numpy as np
import copy
import time

#Pose = np.array([float, float, float])
#       = (x,y,theta)

class RrtStar(object):
    """RRT* search

    The search is incremental. You first 'initialize_search' for a
    given query before calling 'expand_tree'.
    At any time, you can get the 'current_path'.

    Alternatively, the 'search' method is doing it all.

    nodes: array of poses
    parents: dictionary of indexes with key=index of child in nodes
                                        value=index of parent in nodes
    costs: dictionary of costs with key=index of pose in nodes
    """

    def __init__(self, eta, world):
        # parameters
        self._eta = eta
        # internal state
        self.nodes = np.empty((0,3))  # type: NdArray[Pose]
        self.parents = {}  # type: Dict[int, Optional[int]]
        self.costs = {}  # type: Dict[int, float]
        self.gamma = np.inf
        self.goal = None
        self._world = world
        self._initialized = False

    def search(self, start, goal, N=1000):
        # type: (Pose, Pose, int) -> Ndarray[Pose]
        """Search path from start to goal.

        Args:
            start: starting pose (array).
            goal: goal pose (array).
            N: number of samples (default: 1000).

        Return:
            array of poses from start to goal.

        Raise:
            ValueError if no path found.
        """
        self.initialize_search(start, goal)
        self.expand_tree(N)
        current_path = self.current_path
        if current_path is None:
            raise ValueError('Could not find path.')
        return current_path

    @property
    def edges(self):
        # type: () -> Ndarray[Tuple[Pose, Optional[Pose]]]
        """Return tree edges as pose tuples"""
        ind_edges = list(self.parents.items())
        edges = np.empty((0,2,3))
        for ind_tuple in ind_edges:
            if ind_tuple == (0, None):
                continue
            #print(ind_tuple)
            edge_k = np.array([self.nodes[ind_tuple[0]], self.nodes[ind_tuple[1]]])
            #print(edge_k)
            edges = np.append(edges, [edge_k], axis=0)
        return edges

    @property
    def r_near(self):
        # type: () -> None
        """Radius of neighbor search."""
        n = len(self.nodes)
        return min(self._eta, np.sqrt(self.gamma/np.pi * np.log(n)/n))

    def _update_gamma(self):
        # type: () -> None
        """Update the gamma parameter in neighbor search."""
        if self._world is None:
            self.gamma = np.inf
            return
        # higher bound for the free space volume
        config_space_volume = self._world.get_configuration_space_volume()
        gamma_L = 8*(4/3)* config_space_volume
        self.gamma = gamma_L

    def nearest(self, pose):
        # type: (Pose) -> int
        """Get index of nearest node in the tree.

        Args:
            pose: pose (array) to look for nearest neighbor.

        Return:
            nearest pose in the tree.
        """
        assert len(self.nodes)>0, 'No nodes.'
        indexes = [i for i in range(len(self.nodes))]
        closest_index = min(indexes, key=lambda x: self.dist(self.nodes[x], pose))
        return closest_index

    def steer(self, p1, p2):
        # type: (Pose, Pose) -> Pose
        """Find point at distance eta from p1 in direction to p2.

        Args:
            p1: starting pose (array).
            p2: pose (array) to aim for.

        Return:
            pose closest to p2 but not farther away than eta from p1.
        """
        assert self._world is not None, 'Need map to steer.'

        d = self.dist(p1, p2)
        
        # if p2 is close enough, we do not need to steer
        if d <= self._eta:
            return p2
        
        # we need to adjust the theta coordinate of p1
        p1_real = np.copy(p1)
        theta1 = p1[2]
        theta2 = p2[2]
        if abs(theta1 - theta2) > np.pi:
            if theta1 < np.pi:
                p1_real[2] += 2*np.pi
            else:
                p1_real[2] -= 2*np.pi
        
        # compute steered pose
        p = p1_real + self._eta*(p2 - p1_real)/d
        p[2] = p[2] % (2*np.pi)
        
        return p

    def near(self, pose):
        # type: (Pose) -> Set[Pose]
        """Find indexes of neighborhood of given pose.

        The neighborhood size is governed by self.r_near.

        Args:
            pose: the pose around which looking for poses.

        Return:
            indexes of all poses in the tree with distance to pose less than r_near.
        """
        indexes = [i for i in range(len(self.nodes))]
        r_near = self.r_near
        neighborhood = [ind for ind in indexes if self.dist(self.nodes[ind], pose) <= r_near]
        return np.array(neighborhood)

    @staticmethod
    def dist(pose1, pose2):
        # type: (Pose, Pose) -> float
        """Euclidean distance between pose1 and pose2."""

        dpose = pose2 - pose1
        # compute real distance on theta coordinate
        #dpose[2] = min(dpose[2], 2*np.pi - dpose[2])
        dpose[2] = 0
        return np.sqrt(np.sum(dpose**2))

    def initialize_search(self, start, goal):
        # type: (Pose, Pose) -> None
        """Initialize new search.

        Args:
            start: starting pose (array).
            goal: goal pose (array).
        """
        # internal state
        self.goal = goal
        self._update_gamma()
        # initialize search state
        self.nodes = np.append(self.nodes, [start], axis=0)
        self.parents = {0: None}
        self.costs = {0: 0.}
        self._initialized = True

    def expand_tree(self, N=1):
        # type: (int) -> None
        """Add samples to the search tree.

        Args:
            N: number of samples to add (default: 1).
        """
        assert self._initialized, 'Search not initialized.'
        for _ in range(N):
            #t = time.time()
            x_rand = self._world.sample_free()
            ind_nearest = self.nearest(x_rand) # critique
            x_nearest = self.nodes[ind_nearest]
            x_new = self.steer(x_nearest, x_rand)
            #print("1-", time.time()-t)
            if self._world.coll_free(x_nearest, x_new):
                #print("2-", time.time()-t)
                neighbors_ind = self.near(x_new) # critique
                #print("3-", time.time()-t)
                # critique
                ind_near = np.array([i for i in neighbors_ind if self._world.coll_free(self.nodes[i], x_new)])
                #print("4-", time.time()-t)
                cost_min = self.costs[ind_nearest] + self.dist(x_nearest, x_new)
                #print("5-", time.time()-t)
                ind_min = ind_nearest
                #print("11-", time.time()-t)
                for ind in ind_near:
                    cost = self.costs[ind] + self.dist(self.nodes[ind], x_new)
                    if cost < cost_min:
                        cost_min = cost
                        ind_min = ind
                self.nodes = np.append(self.nodes, [x_new], axis=0)
                ind_new = len(self.nodes)-1
                x_min = self.nodes[ind_min]
                self.parents[ind_new] = ind_min
                self.costs[ind_new] = self.costs[ind_min] + self.dist(x_min, x_new)
                
                for ind in ind_near:
                    x = self.nodes[ind]
                    if self.costs[ind_new] + self.dist(x_new, x) < self.costs[ind]:
                        self.parents[ind] = ind_new
                        self.costs[ind] = self.costs[ind_new] + self.dist(x_new, x)

    @property
    def current_path(self):
        # type: () -> Optional[Array[Pose]]
        """Current path."""
        assert self._initialized
        # abort if goal not found
        ind_nearest_from_goal = self.nearest(self.goal)
        x_nearest_from_goal = self.nodes[ind_nearest_from_goal]
        if self.dist(x_nearest_from_goal, self.goal) > self.r_near:
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
