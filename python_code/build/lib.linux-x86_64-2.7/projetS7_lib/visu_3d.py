import numpy as np
from glumpy import app, gl, glm, gloo
from class_3D import Class3D
from rrt_star import RrtStar

class Visu3D:

    def __init__(self, world_filename, tree_filename, 
                WORLDS_DIR, TREES_DIR, OBSTACLES_DIR, ROBOTS_DIR,
                mode='both', obstacles_cloud_filename=None):
        """
        Initialize 3D visualization

        Args:
            world_filename: name of world file
            tree_filename: name of RRT result file
            mode: 'rrt' shows RRT path, 'obstacles_cloud' shows obstacles cloud, 'both' shows both
            obstacles_cloud_filename: name of obstacles cloud filename, leave as None to calculate
        """
        vertex_tree = """
        uniform mat4   u_model;         // Model matrix
        uniform mat4   u_view;          // View matrix
        uniform mat4   u_projection;    // Projection matrix
        attribute vec3 a_position;      // Vertex position
        void main()
        {
            gl_Position = u_projection * u_view * u_model * vec4(a_position,1.0);
        } """

        fragment_tree = """
        void main()
        {
            gl_FragColor = vec4(1, 1.0, 1.0, 1.0);
        } """
        #___________________________Define Fragement and Model________________________
        vertex_cloud = """
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

        fragment_cloud = """
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

        classv = Class3D(world_filename, tree_filename, WORLDS_DIR, TREES_DIR, OBSTACLES_DIR, ROBOTS_DIR)
        if mode == 'obstacles_cloud' or mode == 'both':
            if obstacles_cloud_filename is None:
                self.obstacles_cloud = classv._get_cloud_vertices(classv.world, 10000)
                classv.save_cloud("obstacle_" + world_filename)
            else:
                self.obstacles_cloud = classv.load_cloud(obstacles_cloud_filename)
            vertices, colors, radiuses = classv.draw_occupied(classv.world, self.obstacles_cloud)

        ##############################################################################
        self.window = app.Window(width=1920, height=1200,
                            color=(.6, .7, .7, 1.00), title="Tree Visualiser")


        if mode == 'rrt' or mode == 'both':
            V, I = classv.draw_rrt(classv.tree, classv.path)
            V = V.view(gloo.VertexBuffer)
            I = I.view(gloo.IndexBuffer)

            self.edges_program = gloo.Program(vertex_tree, fragment_tree)
            self.edges_program["a_position"] = V

            view = np.eye(4,dtype=np.float32)
            model = np.eye(4,dtype=np.float32)
            projection = np.eye(4,dtype=np.float32)

            self.x, self.y, self.z=0., 0., 0.
            self.delta_theta = .5
            self.delta_phi = .5

            glm.translate(view, 0, 0,-5)
            self.edges_program['u_model'] = model
            self.edges_program['u_view'] = view
            self.edges_program['u_projection'] = projection
            self.phi, self.theta = 0,0
        ###############################################################################
        if mode == 'obstacles_cloud' or mode == 'both':
            self.vertices_program = gloo.Program(vertex_cloud, fragment_cloud)
            view = np.eye(4, dtype=np.float32)
            glm.translate(view, 0, 0, -5)

            self.vertices_program['position'] = vertices
            self.vertices_program['radius']   =radiuses
            self.vertices_program['fg_color'] = 0,0,0,1
            self.vertices_program['bg_color'] = colors
            self.vertices_program['linewidth'] = 0
            self.vertices_program['antialias'] = 0
            self.vertices_program['u_model'] = np.eye(4, dtype=np.float32)
            self.vertices_program['u_projection'] = np.eye(4, dtype=np.float32)
            self.vertices_program['u_view'] = view

        ###############################################################################

        @self.window.event
        def on_resize(width, height):
            ratio = width / float(height)
            if mode == 'rrt' or mode == 'both':
                self.edges_program['u_projection'] = glm.perspective(45.0, ratio, 2.0, 100.0)
            if mode == 'obstacles_cloud' or mode == 'both':
                self.vertices_program['u_projection'] = glm.perspective(45.0, ratio, 2.0, 100.0)

        @self.window.event
        def on_draw(dt):
            #global phi, theta, translate, z, n
            self.window.clear()
            self.edges_program.draw(gl.GL_LINES, I)
            #self.edges_program.draw(gl.GL_POINTS, V)
            self.vertices_program.draw(gl.GL_POINTS, vertices)
            # Make cube rotate
            self.theta += self.delta_theta # degrees
            self.phi += self.delta_phi # degrees
            model = np.eye(4, dtype=np.float32)
            glm.rotate(model, self.theta, 0, 0, 1)
            glm.rotate(model, self.phi, 0, 1, 0)
            glm.translate(model, self.x, self.y, -self.z)
            self.edges_program['u_model'] = model
            self.vertices_program['u_model'] = model

        @self.window.event
        def on_init():
            gl.glEnable(gl.GL_DEPTH_TEST)

        #######################User interface functions#################################

        @self.window.event
        def on_key_press(symbol, modifiers):
            #global theta, phi, x, y, z
            if symbol == 65362:
                self.theta += 10
            if symbol == 65364:
                self.theta -= 10
            if symbol == 65363:
                self.phi += 10
            if symbol == 65361:
                self.phi -= 10
            if symbol == 122:
                self.y += .2
            if symbol == 115:
                self.y -= .2
            if symbol == 113:
                self.x -= .2
            if symbol == 100:
                self.x += .2

            #print(symbol, modifiers)

        @self.window.event
        def on_mouse_scroll(dx, dy, d1, d2):
            #global z
            if d2 > 0:
                self.z += .5
            elif d2 <0:
                self.z -= .5

        @self.window.event
        def on_mouse_press(x, y, button):
            #global delta_theta, delta_phi
            if button == 1:
                if self.delta_theta == .5:
                    self.delta_theta = 0
                elif self.delta_theta == 0:
                    self.delta_theta = .5
            if button == 4:
                if self.delta_phi == .5:
                    self.delta_phi = 0
                elif self.delta_phi == 0:
                    self.delta_phi = .5
            if button == 2:
                if self.delta_phi != 0 or self.delta_theta !=0:
                    self.delta_phi = 0
                    self.delta_theta = 0
                else:
                    self.delta_theta = .5
                    self.delta_phi = .5
            print(x, y, button)

        app.run()