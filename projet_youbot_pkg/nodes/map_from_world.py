from st5generic.grid_map import GridMap
from st5_generic.geometry  import *
from world import World
from shapely.geometry import Polygon,Point

class BuildMapFromWorld():
    def __init__(self,world,resolution,inflate_radius ):
        """
        args: 
        world: World object
        resolution: Float        
        inflate_radius:min safety distance to obstacles (Float)
        """
        self.inflate_radius=inflate_radius
        self.resolution=resolution
        self.world=world

    def map_from_world(self):
        """
        calculates Occupancy grip from a world

        returns:
        OccupancyGrid
        """
        
        (x_min,x_max,y_min,y_max))=self.world.get_bounding_box()
        init_size=self.resolution*max(x_max-x_min,y_max-y_min)
        map_grid=GridMap(resolution,init_size,growth=init_size)
        for i in range(init_size+1):
            for j in range(init_size+1):
                x,y=map_grip.index_to_coords(i,j)
                point=Point(x,y)
                for obs in self.world.obstacles:
                    if point.intersects(obs):
                        map_grid.set_cell(i,j,1)
                    else:
                        map_grid.set_cell(i,j,0)
        inflated_map_grid=map_grid.inflate(self.inflate_radius)
        return inflated_map_grid


