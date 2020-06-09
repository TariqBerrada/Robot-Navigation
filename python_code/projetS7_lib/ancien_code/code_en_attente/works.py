import numpy as np
from glumpy import app, gl, glm, gloo
from glumpy.geometry import colorcube
import genLines
vertex = """
uniform mat4   u_model;         // Model matrix
uniform mat4   u_view;          // View matrix
uniform mat4   u_projection;    // Projection matrix
attribute vec3 a_position;      // Vertex position
void main()
{
    gl_Position = u_projection * u_view * u_model * vec4(a_position,1.0);
} """

fragment = """
void main()
{
    gl_FragColor = vec4(0, 1.0, 1.0, 1.0);
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
window = app.Window(width=1920, height=1200,
                    color=(0.6, 0.60, 0.6, 1.00), title="Graph Visualiser")

V = np.zeros(8, [("a_position", np.float32, 3)])
V["a_position"] = [[ 1, 1, 1], [-1, 1, 1], [-1,-1, 1], [ 1,-1, 1],
                   [ 1,-1,-1], [ 1, 1,-1], [-1, 1,-1], [-1,-1,-1]]

I = np.array([0,1, 1,2, 2,3, 3,0, 4,7, 7,6,
              6,5, 5,4, 0,5, 1,6, 2,7, 3,4], dtype=np.uint32)


V = V.view(gloo.VertexBuffer)
I = I.view(gloo.IndexBuffer)

lines = gloo.Program(vertex, fragment)
lines["a_position"] = V

view = np.eye(4,dtype=np.float32)
model = np.eye(4,dtype=np.float32)
projection = np.eye(4,dtype=np.float32)

x, y, z=0., 0., 0.

glm.translate(view, 0, 0,-5)
lines['u_model'] = model
lines['u_view'] = view
lines['u_projection'] = projection
phi, theta = 0,0
########################Create Your Second Model###############################
#For some reason only works correctly if h>=4
n = 1000
h = 8

poly2d = genLines.add_polygon_list()
poly3d = []
for i in range(len(V["a_position"])):
    poly3d.append(V["a_position"][i])
for i in range(len(poly2d)):
    xy = poly2d[i]
    for j in range(h):
        poly3d.append(poly2d[i]+[float(j)/float(h)])
#print(poly3d[:3])
poly3d = np.array(poly3d)
program = gloo.Program(vertex_cloud, fragment_cloud)
view = np.eye(4, dtype=np.float32)
glm.translate(view, 0, 0, -5)

program['position'] = poly3d
program['radius']   = np.array([10]*8 + [5 for i in range(n*h)])
program['fg_color'] = 0,0,0,1
colors = 0.2*np.ones((n*h, 4), dtype=np.float32)
colors[:,3] = 1
#print(colors)
program['bg_color'] = colors
#print(program['fg_color'])
program['linewidth'] = 1.0
program['antialias'] = 1
program['u_model'] = np.eye(4, dtype=np.float32)
program['u_projection'] = np.eye(4, dtype=np.float32)
program['u_view'] = view
###############################################################################

@window.event
def on_resize(width, height):
   ratio = width / float(height)
   lines['u_projection'] = glm.perspective(45.0, ratio, 2.0, 100.0)
   program['u_projection'] = glm.perspective(45.0, ratio, 2.0, 100.0)

delta_theta = .5
delta_phi = .5
@window.event
def on_draw(dt):
    global phi, theta, translate, z, n
    window.clear()
    lines.draw(gl.GL_LINES, I)
    #lines.draw(gl.GL_POINTS, V)
    program.draw(gl.GL_POINTS, poly3d)
    # Make cube rotate
    theta += delta_theta # degrees
    phi += delta_phi # degrees
    model = np.eye(4, dtype=np.float32)
    glm.rotate(model, theta, 0, 0, 1)
    glm.rotate(model, phi, 0, 1, 0)
    glm.translate(model, x, y, -z)
    lines['u_model'] = model
    program['u_model'] = model
    #Making some funny animations right in here
    V["a_position"][0] = [1+.25*np.cos(phi/10), 1+.25*np.sin(phi/10), 1]
    #program['position'][21] = ([0.7911187 , 0.08529159, .25*np.cos(phi/10)])
    #print(len(poly3d))
    program['position'][0] = [1+.25*np.cos(phi/10), 1+.25*np.sin(phi/10), 1]
    
    

@window.event
def on_init():
    gl.glEnable(gl.GL_DEPTH_TEST)

#################Add your user interface fonctions right here#################
@window.event
def on_key_press(symbol, modifiers):
    global theta, phi, x, y, z
    if symbol == 65362:
        theta += 10
    if symbol == 65364:
        theta -= 10
    if symbol == 65363:
        phi += 10
    if symbol == 65361:
        phi -= 10

    if symbol == 90:
        y += .2
    if symbol == 83:
        y -= .2
    if symbol == 81:
        x -= .2
    if symbol == 68:
        x += .2

    print(symbol, modifiers)

@window.event
def on_mouse_scroll(dx, dy, d1, d2):
    global z
    if d2 > 0:
        z -= 1
    elif d2 <0:
        z+=1

@window.event
def on_mouse_press(x, y, button):
    global delta_theta, delta_phi
    if button == 4:
        if delta_theta == .5:
            delta_theta = 0
        elif delta_theta == 0:
            delta_theta = .5
    if button == 8:
        if delta_phi == .5:
            delta_phi = 0
        elif delta_phi == 0:
            delta_phi = .5
    if button == -1:
        if delta_phi != 0 or delta_theta !=0:
            delta_phi = 0
            delta_theta = 0
        else:
            delta_theta = .5
            delta_phi = .5
    print(x, y, button)

app.run()