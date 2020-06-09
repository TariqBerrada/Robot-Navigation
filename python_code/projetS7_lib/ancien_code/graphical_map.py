import triangle
from shapely.geometry import Point, Polygon
from shapely import affinity
from world import World
import numpy as np
from glumpy import app, gl
from glumpy.graphics.collections import TriangleCollection, PathCollection
from glumpy.transforms import Position, OrthographicProjection, PanZoom

class GraphicalMap(object):
    def __init__(self,array_of_polygons,drone_polygon,initial_pose,resolution):
        """parameters:
        list_of_polygons: array of Polygon objects
        drone_polygon: Polygon object
        resolution: Int
        initial_pose: Array(len=3)"""
        self.array_of_polygons=array_of_polygons
        self.drone_polygon=drone_polygon
        self.resolution=resolution
        self.initial_pose=initial_pose

#list_of_polygons=[Polygon(((0.,0.),(0.,0.5),(0.5,0.5),(0.5,0.),(0.,0.))),Polygon(((0.8,0.8),(0.8,1.),(1.,1.),(1.,0.8),(0.8,0.8)))]
#drone_polygon=Polygon(((0.1,0.),(0.,0.1),(0.,0.),(0.1,0.)))
#initial_pose=[0.8,0.5,0.]
#initial_pose_np=np.array(initial_pose)
#resolution=1000
    def polygon_to_array(self, P):                         #conversion en array
        P_interm=list(P.exterior.coords)
        P_array=[]
        for (x,y) in P_interm:
            P_array.append(np.array([x,y,0.]))        #2d mais necessite d etre au format (5,3)
        P_array=np.array(P_array)
        return P_array

    def triangulate(self,P_array):
        n = len(P_array)
        S = np.repeat(np.arange(n+1),2)[1:-1]
        S[-2:] = n-1,0
        S = S.reshape((-1, 2))              #creates the segments
        T = triangle.triangulate({'vertices': P_array[:,:2], 'segments': S}, "p")
        return  T["triangles"].ravel()
    def open_window(self):
        window = app.Window(self.resolution, self.resolution, color=(1,1,1,1))
        @window.event
        def on_draw(dt):
            window.clear()
            self.triangles.draw()
            self.paths.draw()

        @window.event
        def on_init():
            gl.glEnable(gl.GL_DEPTH_TEST)

        @window.event
        def on_key_press(key, modifiers):
            if key == app.window.key.SPACE:
                transform.reset()

        transform = PanZoom(OrthographicProjection(Position()))
        self.triangles = TriangleCollection("agg", transform=transform, color='shared')
        self.paths = PathCollection("agg", transform=transform, color='shared')
        self.paths["linewidth"] = 10

        self.world_map=World(self.array_of_polygons, self.drone_polygon,self.initial_pose)
        self.polyg_array=self.world_map.get_polygon_array()
        for pol in self.polyg_array:
            
            P=self.polygon_to_array(pol)
            I = self.triangulate(P)
            self.triangles.append(P*self.resolution, I, color=(0.1,0.1,0.1,1))
            self.paths.append(P*self.resolution, closed=True, color=(0,0,0,1))
        self.robot_pose_array=self.polygon_to_array(self.drone_polygon)+self.initial_pose
        self.robot_I=self.triangulate(self.robot_pose_array)
        self.triangles.append(self.robot_pose_array*self.resolution,self.robot_I,color=(0.8,0.2,0,1))
        self.paths.append(self.robot_pose_array*self.resolution,closed=True,color=(0,0,1,1))


        window.attach(self.paths["transform"])
        window.attach(self.paths["viewport"])
        app.run()

b=GraphicalMap(
np.array([
Polygon(((0.,0.),(0.,0.5),(0.5,0.5),(0.5,0.),(0.,0.))),
Polygon(((0.8,0.8),(0.8,1.),(1.,1.),(1.,0.8),(0.8,0.8)))
]),
Polygon(((0.1,0.),(0.,0.1),(0.,0.),(0.1,0.))),
np.array([0.8,0.5,0.]),
1000)
b.open_window()

