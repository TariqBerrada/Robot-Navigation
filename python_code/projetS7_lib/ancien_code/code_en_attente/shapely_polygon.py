from shapely.geometry import Polygon, Point
import random as rd
import shapely
import matplotlib.pyplot as plt
point = Point(0., 0.)
polygon = Polygon([(0, 0), (1, 1), (1, 0)])
polygon2 = Polygon([(0, 1), (0, .4), (.2, .9), (.4, .1)])
print(polygon.contains(point))
n= 10000
Points_x = []
Points_y = []
for i in range(n):
    x, y = rd.random(), rd.random()
    pot_point = Point(x, y)
    if polygon2.contains(pot_point):
        Points_x.append(x)
        Points_y.append(y)
plt.scatter(Points_x, Points_y)
plt.show()