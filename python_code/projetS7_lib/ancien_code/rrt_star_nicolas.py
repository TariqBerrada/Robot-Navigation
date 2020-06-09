"""
RRT* search
"""

from world import World
import numpy as np
from shapely.geometry import Point, Polygon

class RrtStar(object):
    """RRT* search

    The search is incremental. You first 'initialize_search' for a
    given query before calling 'expand_tree'.
    At any time, you can get the 'current_path'.

    Alternatively, the 'search' method is doing it all.
    """

    def __init__(self, eta, world,N=1000):
        # parameters
        self._eta = eta
        # internal state
        self.nodes = []  # type: List[Pose]
        self.parents = []  # type: Array[Array]], contains 1 in (i,j) if cell of index j is parent of cell of index i
        self.costs = []  # type: Array[Tuple], the tuple contains the pose and its cost(float)
        self.gamma = np.inf
        self.goal = None
        self.N=N            #number of samples
        self.index=0        #index of the node that the algorithm adds to the tree
        self._world = world
        self._initialized = False

    def search(self, start, goal):
        # type: (Pose, Pose, int) -> Ndarray[Pose]
        """Search path from start to goal.

        Args:
            start: starting pose (array).
            goal: goal pose (array).
            
        Return:
            array of poses from start to goal.

        Raise:
            ValueError if no path found.
        """
        self.initialize_search(start, goal)
        self.expand_tree(self.N)
        z=self.costs !=0              # delete elements equal to 0 at the end of costs 
        self.costs=self.costs[z]      #  (is probably not the quickest way to do)
        current_path = self.current_path
        if current_path is None:
            raise ValueError('Could not find path.')
        return current_path

    @property
    def edges(self):
        # type: () -> Ndarray[Tuple[Pose, Optional[Pose]]]
        """Return tree edges."""
        return np.array(list(self.parents.items()))

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

    def sample_free(self):
        # type: () -> Pose
        """Sample a pose in free space."""
        assert self._world is not None
        return self._world.sample_free()

    def nearest(self, pose):
        # type: (Pose) -> Pose
        """Get nearest node in the tree.

        Args:
            pose: pose (array) to look for nearest neighbor.

        Return:
            nearest pose in the tree.
        """
        assert (self.nodes), 'No nodes.'
        closest = min(self.nodes, key=lambda x: self.dist(x, pose))
        return closest

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
        if d <= self._eta:
            p2[2] = p2[2] % (2*np.pi)
            return p2
        
        p1_real = np.copy(p1)
        theta1 = p1[2]
        if abs(theta1 - p2[2]) > np.pi:
            if theta1 < np.pi:
                p1_real[2] += 2*np.pi
            else:
                p1_real[2] -= 2*np.pi
        
        p = p1_real + self._eta*(p2 - p1_real)/d
        p[2] = p[2] % (2*np.pi)
        
        return p

    def coll_free(self, p1, p2, steps=10):
        # type: (Pose, Pose) -> bool
        """Check line of sight between poses.

        Args:
            p1: first pose (array).
            p2: second pose (array).

        Return:
            True if no collision, False otherwise.
        """
        assert self._world is not None, 'Need map to check collision.'
        return self._world.coll_free(p1, p2, steps=steps)

    def near(self, pose):
        # type: (Pose) -> Set[Pose]
        """Find neighborhood of given pose.

        The neighborhood size is governed by self.r_near.

        Args:
            pose: the pose around which looking for poses.

        Return:
            all the poses in the tree with distance to pose less than r_near.
        """
        return [p for p in self.nodes if self.dist(p, pose) <= self.r_near]

    @staticmethod
    def dist(pose1, pose2):
        # type: (Pose, Pose) -> float
        """Euclidean distance between pose1 and pose2."""
        dpose = pose2 - pose1
        dpose[2] = min(dpose[2], 2*np.pi - dpose[2])
        return np.sqrt(np.sum(dpose**2))

    def research_index(self,matrix,array_to_find):
        # type: (Array,Array) -> int
        """finds the firrst occurence index of the array in the matrix (only method found with Numpy syntax)"""
        for i,element in enumerate(matrix):
            if not (element-array_to_find).any():
                return i
        raise ValueError("Could not find array in the matrix")

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
        self.nodes = []
        self.nodes.append(start)
        self.parents = np.zeros( (self.N,self.N) )  # if b is parent of a, self.parents[index_of_a,index_of_b]=1 
        self.costs = np.zeros(self.N,dtype=tuple)  
        self.costs[0]=(start,0)
        self._initialized = True

    def expand_tree(self, N=1):
        # type: (int) -> None
        """Add samples to the search tree.

        Args:
            N: number of samples to add (default: 1).
        """
        assert self._initialized, 'Search not initialized.'
        for _ in range(N):    
            x_rand = self.sample_free()
            x_nearest = self.nearest(x_rand)
            x_new = self.steer(x_nearest, x_rand)
            if self.coll_free(x_nearest, x_new):
                self.index+=1
                X_near = [x for x in self.near(x_new) if self.coll_free(x, x_new)]
                cost_min = self.costs[self.research_index(self.nodes,x_nearest)][1] + self.dist(x_nearest, x_new)
                x_min = x_nearest
                for x in X_near:
                    cost = self.costs[self.research_index(self.nodes,x)][1] + self.dist(x, x_new)
                    if cost < cost_min:
                        cost_min = cost
                        x_min = x
                
                self.nodes.append(x_new)
                j=self.research_index(self.nodes,x_min)
                self.parents[self.index,j]=1
                self.costs[self.index] = (x_new,self.costs[j][1] + self.dist(x_min, x_new))
                for x in X_near:
                    k=self.research_index(self.nodes,x)
                    if self.costs[self.index][1] + self.dist(x_new, x) < self.costs[k][1]:
                        self.parents[self.index]=np.zeros(self.N)
                        self.parents[self.index,k] = 1
                        self.costs[k] = (self.costs[k][0],self.costs[self.index][1] + self.dist(x_new, x))
                        

    @property
    def current_path(self):
        # type: () -> Optional[Array[Pose]]
        """Current path."""
        assert self._initialized
        # abort if goal not found
        x_nearest_from_goal = self.nearest(self.goal)
        if self.dist(x_nearest_from_goal, self.goal) > self.r_near:
            return None
        
        # build path by backtracking from goal to start
        else:
            path = np.array([self.goal,x_nearest_from_goal])
            x = x_nearest_from_goal
            while (x-self.nodes[0]).any():
                
                #j=self.parents[self.nodes.index(x)].index(1)
                j=np.where(self.parents[self.research_index(self.nodes,x)]==1)[0][0]
                x = self.nodes[j]
                
                path = np.append(path, np.array([x]),0)
            path = np.flip(path,0)
        return path




my_robot = Polygon(((40, 40), (40, 100), (60, 100), (60, 60),(100, 60), (100, 100), (120, 100), (120, 40)))
my_obstacles = np.array([Polygon(( (240, 130), (380, 230), (10, 280), (240, 130)) ), Polygon(((500, 130), (720, 109), (620, 100), (640, 125),(500, 130) ))])
my_pose = np.array([0, 0, 0])
my_world = World(my_obstacles, my_robot)
my_goal=np.array([150,150,0])
a=RrtStar(50,my_world,N=1000)           #N=1000: takes between 10 and 30 sec
a.initialize_search(my_pose, my_goal )

a.search(my_pose, my_goal)

print(a.current_path)
