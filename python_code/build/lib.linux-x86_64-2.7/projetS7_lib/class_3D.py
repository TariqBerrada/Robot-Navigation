import yaml
import numpy as np
from shapely.geometry import Polygon
from world import World
from glumpy import app, gl, glm, gloo
import os

class Class3D():
    #In some cases, the absolute coordinates of the directory must be specified in order for the YAML files to be read.
    """CLASS_DIR = os.path.dirname(os.path.realpath(__file__))
    WORLDS_DIR = CLASS_DIR + '/worlds/'
    TREES_DIR =  CLASS_DIR + '/trees/'
    OBSTACLES_DIR = CLASS_DIR + '/3D_obstacles/'"""
    #window = app.Window(width=1920, height=1200,
    #                        color=(0.6, 0.60, 0.6, 1.00), title="Graph Visualiser")
    def __init__(self, world_filename, tree_filename, WORLDS_DIR, TREES_DIR, OBSTACLES_DIR, ROBOTS_DIR):
        self.WORLDS_DIR = WORLDS_DIR
        self.TREES_DIR = TREES_DIR
        self.OBSTACLES_DIR = OBSTACLES_DIR
        self.world = World(world_filename, WORLDS_DIR, ROBOTS_DIR)
        
        # Load dictionary from yaml file describing world
        stream = open(self.TREES_DIR + tree_filename + ".yaml")
        loaded_dict = yaml.load(stream)
        stream.close()
        self.tree = loaded_dict['edges']
        self.path = loaded_dict['path']
        self.leafs = []
        self.obstacles_cloud = None

        """self.x, self.y, self.z = 0., 0., 0.
        self.phi, self.theta = 0,0
        self.delta_theta = .5
        self.delta_phi = .5"""
        #Initializing elemnts that will upload to the GPU.
        self.vertex_tree = """
        uniform mat4   u_model;         // Model matrix
        uniform mat4   u_view;          // View matrix
        uniform mat4   u_projection;    // Projection matrix
        attribute vec3 a_position;      // Vertex position
        void main()
        {
            gl_Position = u_projection * u_view * u_model * vec4(a_position,1.0);
        } """

        self.fragment_tree = """
        void main()
        {
            gl_FragColor = vec4(0, 0, 0, 1);
        } """
        
        self.vertex_cloud = """
        #version 120
        uniform mat4 u_model;
        uniform mat4 u_view;
        uniform mat4 u_projection;
        uniform float linewidth;
        uniform float antialias;
        attribute vec4  fg_color;
        attribute vec4  bg_color;
        attribute float radius;
        attribute vec3  position;
        varying float v_pointsize;
        varying float v_radius;
        varying vec4  v_fg_color;
        varying vec4  v_bg_color;
        void main()
        {
            v_radius = radius;
            v_fg_color = fg_color;
            v_bg_color = bg_color;
            gl_Position = u_projection * u_view * u_model * vec4(position,1.0);
            gl_PointSize = 2 * (v_radius + linewidth + 1.5*antialias);
        }
        """

        self.fragment_cloud = """
        #version 120
        uniform float linewidth;
        uniform float antialias;
        varying float v_radius;
        varying vec4  v_fg_color;
        varying vec4  v_bg_color;
        float marker(vec2 P, float size)
        {
        const float SQRT_2 = 1.4142135623730951;
        float x = SQRT_2/2 * (P.x - P.y);
        float y = SQRT_2/2 * (P.x + P.y);
        float r1 = max(abs(x)- size/2, abs(y)- size/10);
        float r2 = max(abs(y)- size/2, abs(x)- size/10);
        float r3 = max(abs(P.x)- size/2, abs(P.y)- size/10);
        float r4 = max(abs(P.y)- size/2, abs(P.x)- size/10);
        return min( min(r1,r2), min(r3,r4));
        }
        void main()
        {
            float r = (v_radius + linewidth + 1.5*antialias);
            float t = linewidth/2.0 - antialias;
            float signed_distance = length(gl_PointCoord.xy - vec2(0.5,0.5)) * 2 * r - v_radius;
        //    float signed_distance = marker((gl_PointCoord.xy - vec2(0.5,0.5))*r*2, 2*v_radius);
            float border_distance = abs(signed_distance) - t;
            float alpha = border_distance/antialias;
            alpha = exp(-alpha*alpha);
            // Inside shape
            if( signed_distance < 0 ) {
                // Fully within linestroke
                if( border_distance < 0 ) {
                    gl_FragColor = v_fg_color;
                } else {
                    gl_FragColor = mix(v_bg_color, v_fg_color, alpha);
                }
            // Outside shape
            } else {
                // Fully within linestroke
                if( border_distance < 0 ) {
                    gl_FragColor = v_fg_color;
                } else if( abs(signed_distance) < (linewidth/2.0 + antialias) ) {
                    gl_FragColor = vec4(v_fg_color.rgb, v_fg_color.a * alpha);
                } else {
                    discard;
                }
            }
        }
        """

        ##############################################################################

        self.vertices_program = None
        self.edges_program = None
        
    def _get_relevant_bounding_box(self, obstacle):
        """returns the region of space where the points that could represent the occupied space around an obstacle are going to be sampled
        args:
            obstacle: Polygon
        """
        minx, miny, maxx, maxy = obstacle.bounds
        relevant_diameter = np.sqrt((self.world.bounding_box[0]-self.world.bounding_box[2])**2+(self.world.bounding_box[1]-self.world.bounding_box[3])**2)
        return minx-relevant_diameter, miny-relevant_diameter, maxx+relevant_diameter, maxy+relevant_diameter

    def check_collision(self, obstacle, pose):
        """
        args:
            obstacle: Polygon
            pose: array
        """
        simulated_robot = self.world.get_robot(pose)
        x = simulated_robot.intersects(obstacle)
        return x

    def _get_cloud_vertices(self, world, density_obs):
        """draws nb_pts points in the graph, if they fit in the occupied space.

        args:
            world: World
            density_obs: Int -- number of points per arbitrary unit of volume

        """
        print("Generating obstacle clouds...")
        obstacles_cloud = []
        for i in range(len(self.world.obstacles)):
            print(str(i+1) + " / " + str(len(self.world.obstacles)))
            obstacle = Polygon(self.world.obstacles[i])
            obstacle_bounding_box = self._get_relevant_bounding_box(obstacle)
            n_iter = int(obstacle.area * density_obs)
            j = 0
            while j < n_iter:
                x, y, theta = np.random.uniform(obstacle_bounding_box[0], obstacle_bounding_box[2]), np.random.uniform(obstacle_bounding_box[1], obstacle_bounding_box[3]), np.random.uniform(0, 2*np.pi)
                if self.check_collision(obstacle, [x, y, theta]) == True:
                    obstacles_cloud.append([x, y, theta])
                    j += 1
        self.obstacles_cloud = obstacles_cloud
        return obstacles_cloud

    def save_cloud(self, filename):
        """
        Saves the obtacles cloud into a YAML file.
        """
        dict_to_dump = {
            'obstacles': self.obstacles_cloud

        }
        stream = open(self.OBSTACLES_DIR + filename + ".yaml", 'w')
        yaml.dump(dict_to_dump, stream)
        stream.close()
    
    def load_cloud(self, filename):
        """
        Loads an obstacles cloud from a yaml file
        """
        stream = open(self.OBSTACLES_DIR + filename + ".yaml", 'r')
        loaded_dict = yaml.load(stream)
        stream.close()

        return (
            loaded_dict['obstacles']
        )

    def draw_occupied(self, world, obstacles_cloud, with_tree = True):
        """
        args:
            world: world
            with_path : Boolean specifying whether or not the tree vertices are to be drawn.
        """
        vertices = []
        if with_tree:
            self._update_leafs()
            vertices += self.leafs
        vertices += obstacles_cloud
        vertices_nbre = len(vertices)
        cloud_nbre = len(obstacles_cloud)
        vertices = np.array(vertices)
        self.vertices_program = gloo.Program(self.vertex_cloud, self.fragment_cloud)
        view = np.eye(4, dtype=np.float32)
        glm.translate(view, 0, 0, -5)

        self.vertices_program['position'] = vertices
        self.vertices_program['radius']   = np.array([4]*len(self.leafs) + [2 for i in range(cloud_nbre)])
        self.vertices_program['fg_color'] = 0,0,0,1
        colors = 0.2*np.ones((vertices_nbre, 4), dtype=np.float32)
        colors[0:len(self.leafs)] = [ [.8, .9, .9, 1] for i in range(len(self.leafs))]
        colors[len(self.leafs):,3] = .3
        #colors[:len(self.leafs),3] = 1
        return vertices, colors, self.vertices_program['radius']

    def _update_leafs(self):
        leafs = []
        for edge in self.tree:
            if edge[0] not in leafs:
                leafs.append(edge[0])
            if edge[1] not in leafs:
                leafs.append(edge[1])
        self.leafs = leafs

    def draw_rrt(self,edges,path):
        """
        draws rrt tree and the path

        args:
        edges: list of doubles of poses
        path: sorted list of poses from start to goal
        """

        V = np.zeros(2*len(self.tree), [("a_position", np.float32, 3)])

        V["a_position"] = [vertex for edge in self.tree for vertex in edge ]
        indexes = [i for i in range(len(V["a_position"]))]

        I = np.array(indexes, dtype=np.uint32)
        return V, I
