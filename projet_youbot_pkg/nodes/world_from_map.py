from st5generic.grid_map import GridMap
from st5_generic.geometry  import *
from world import World
import shapely
import numpy as np
import cv2
from shapely.geometry import Polygon,Point, LineString

import random as rd

class BuildWorldFromMap():
    def __init__(self,map0,robot_poly):
        self.map = map0
        self.robot_pose = robot_poly
        self.image = self.GridMapToImage(self.map)
        self.shape = self.image.shape

    def GridMapToImage(self, map0):
        image = np.zeros(map0.shape)
        for i in range(map0.shape[0]):
            for j in range(map0.shape[1]):
                image[i][j] = 255*map0.get_cell(i,j)
        return image

    def get_polygon_list(self, image):
        """Takes an image of a polygon that was created from the map and returns a corresponding Shapely Polygon"""
        image = self.image

        def draw_contour(image):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edged = cv2.Canny(blurred, 10, 80, 255)
            return edged

        def detect_corners(image):
            ret,GridMap = cv2.threshold(image,40,255,cv2.THRESH_BINARY_INV)
            blur = cv2.GaussianBlur(GridMap,(5,5),0)
            gray = cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
            # find Harris corners
            gray = np.float32(gray)
            dst = cv2.cornerHarris(gray,3,1,0.04)
            dst = cv2.dilate(dst,None)
            ret, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
            dst = np.uint8(dst)

            # find centroids
            ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

            # define the criteria to stop and refine the corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            corners = cv2.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)
            return corners

        def detect_corners2(image):
            ret,GridMap = cv2.threshold(image ,40, 255, cv2.THRESH_BINARY_INV)
            gray = cv2.cvtColor(GridMap,cv2.COLOR_BGR2GRAY)
            corners = cv2.goodFeaturesToTrack(gray,25,0.01,10)
            corners = np.int0(corners)
            corners2 = [tuple(corner[0]) for corner in corners]
            return corners2

        def get_equation(pt1, pt2):
            """find an equation of type ax+by+c=0"""
            if pt1[0] == pt2[0]:
                b = 0
                a = 1
                c = -pt1[0]
            elif pt1[1] == pt2[1]:
                b = 1
                a = 0
                c = -pt1[1]
            else:
                a = -(pt2[1] - pt1[1])/(pt2[0] - pt1[0])
                b = 1
                c = (-a*(pt1[0]+pt2[0]) - b*(pt1[1]+pt2[0]))/2
            norm_factor = 1/np.sqrt(a**2+b**2)
            a = a*norm_factor
            b = b*norm_factor
            c = c*norm_factor
            return a,b,c

        def get_edge_length(pt1, pt2):
            return np.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

        def get_line_cells(pt1, pt2):
            a, b, c = get_equation(pt1, pt2)
            pixels_list = [tuple(pt1)]
            if np.abs(a)<np.abs(b):
                for i in range(int(pt1[0]+1), int(pt2[0])):
                    pixels_list.append((i, (-c-a*i)/b))
            elif np.abs(b)<np.abs(a):
                for i in range(int(pt1[1]+1), int(pt2[1])):
                    pixels_list.append(((-c-b*i)/a, i))
            pixels_list.append(tuple(pt2))
            return pixels_list

        def get_line_cells2(pt1, pt2):
            canvas = np.zeros(self.shape[:2])
            cv2.line(canvas, pt1, pt2, 255)
            cells = []
            for i in range(canvas.shape[0]):
                for j in range(canvas.shape[1]):
                    if canvas[i, j] == 255:
                        cells.append((i, j))
            return cells

        def get_cells_around(cell):
            cells_list = [cell]
            if 10<cell[0]<self.shape[0]-10 and 10<cell[1]<self.shape[1]-10: 
                cells_list = []
                x = [cell[0] + i for i in range(-4, 5)]
                y = [cell[1] + i for i in range(-4, 5)]
                for s1 in x:
                    for s2 in y:
                        cells_list.append((s1, s2))
            return cells_list

        def get_edges(corners, edged_image, certainty=0.95):
            edges_list = []
            probas = []
            for corner1 in corners:
                for corner2 in corners:
                    if (corner1[0] != corner2[0] or corner1[1] != corner2[1]):
                        cells = get_line_cells2(corner1, corner2) 
                        proba = 0
                        for cell in cells:
                            neighbor_cells = get_cells_around(cell)
                            for cell1 in neighbor_cells:
                                if edged_image[int(cell1[0]), int(cell1[1])] == 255:
                                    proba += 1
                                    break
                        proba = float(proba)/float(len(cells)+1)
                        probas.append(proba)
                        
                        
                        if proba > certainty:
                            edges_list.append([tuple(corner1), tuple(corner2)])
            return edges_list

        def subdivide_edges(edges):
            subdivided_edges = edges
            vertices = set([corner for edge in edges for corner in edge])
            change = True
            while change == True:
                change = False
                for i in range(len(edges)):
                    edge = edges[i]
                    for vertex in vertices:
                        vertex_poly = Point(0, 0).buffer(3)
                        line =   LineString(edge)
                        line_cells1 = get_line_cells2(edge[0], vertex)
                        line_cells2 = get_line_cells2(vertex, edge[1])
                        p1 = 0
                        p2 = 0
                        for cell in line_cells1:
                            neighbor_cells = get_cells_around(cell)
                            for cell1 in neighbor_cells:
                                if edged_image[int(cell1[0]), int(cell1[1])] == 255:
                                    p1 += 1
                                    break
                        p1 = float(p1)/float(len(line_cells1)+1)

                        for cell in line_cells2:
                            neighbor_cells = get_cells_around(cell)
                            for cell1 in neighbor_cells:
                                if edged_image[int(cell1[0]), int(cell1[1])] == 255:
                                    p1 += 1
                                    break
                        p2 = float(p2)/float(len(line_cells2)+1)
                        threshold = .9
                        if vertex not in edge and vertex_poly.intersects(line):
                            if p1 > threshold:
                                subdivided_edges.append([edge[0], vertex])
                            if p2 > threshold:
                                subdivided_edges.append([vertex, edge[1]])
                            del(subdivided_edges[i])
                            change = True
            unique_subdivided_edges = [list(x) for x in set(tuple(x) for x in subdivided_edges)]
            return unique_subdivided_edges

        def calculate_occurences(edges):
            dicti = dict()
            corners  = set([corner for edge in edges for corner in edge])
            for corner in corners:
                dicti[corner] = 0
            for edge in edges:
                dicti[edge[0]] = dicti[edge[0]] + 1
                dicti[edge[1]] = dicti[edge[1]] + 1
            return dicti

        def get_nbre_iterations(edges, error):
            corners  = set([corner for edge in edges for corner in edge])
            vertices = [edges[0][0], edges[0][1]]
            black_list = []
            k = 0
            x= 0
            while k <= len(edges)-2:
                if len(vertices) < 2:
                    k+=1
                    vertices = [edges[k][0], edges[k][1]]
                    black_list = []
                x = max(x, len(vertices))
                found = False
                for edge in edges:
                    if edge[0] == vertices[-1] and edge[1] not in vertices and edge[1] not in black_list:
                        
                        found = True

                        vertices.append(edge[1])
                        break
                    elif edge[1] == vertices[-1] and edge[0] not in vertices and edge[0] not in black_list:    
                        found = True
                        vertices.append(edge[0]) 
                        break
                if found == False:
                    black_list.append(vertices[-1])
                    del(vertices[-1])
            return x

        def order_vertices(edges, error):
            corners  = set([corner for edge in edges for corner in edge])
            vertices = [edges[0][0], edges[0][1]]
            black_list = []
            k = 0
            x = get_nbre_iterations(edges, error)
            while len(vertices) <=x-1:
                if len(vertices) < 2:
                    k+=1
                    vertices = [edges[k][0], edges[k][1]]
                    black_list = []
                found = False
                
                for edge in edges:
                    if edge[0] == vertices[-1] and edge[1] not in vertices and edge[1] not in black_list:
                        
                        found = True

                        vertices.append(edge[1])
                        break
                    elif edge[1] == vertices[-1] and edge[0] not in vertices and edge[0] not in black_list:
                        
                        found = True
                        
                        vertices.append(edge[0]) 
                        break
                if found == False:
                    black_list.append(vertices[-1])
                    del(vertices[-1])
            return vertices

        def order_vertices_multi(edges0, error):
            edges = edges0
            previous_nbre = -1
            polygon_list = []
            while len(edges) != previous_nbre and len(edges) >=3:
                vertices = [edges[0][0], edges[0][1]]
                black_list = []
                k = 0
                x = get_nbre_iterations(edges, error)
                while len(vertices) <=x-1:
                    if len(vertices) < 2:
                        k+=1
                        vertices = [edges[k][0], edges[k][1]]
                        black_list = []
                    found = False    
                    for edge in edges:
                        if edge[0] == vertices[-1] and edge[1] not in vertices and edge[1] not in black_list:
                            
                            found = True

                            vertices.append(edge[1])
                            break
                        elif edge[1] == vertices[-1] and edge[0] not in vertices and edge[0] not in black_list:
                            
                            found = True
                            
                            vertices.append(edge[0]) 
                            break
                    if found == False:
                        black_list.append(vertices[-1])
                        del(vertices[-1])

                previous_nbre = len(edges)
                edges_to_keep = []
                edges_to_delete = set()
                for i in range(len(edges)):
                    
                    for vertex in vertices:
                        if edges[i][0] == vertex or edges[i][1] == vertex:
                            edges_to_delete.add(i)
                            break
                    
                
                edges_to_keep = [edges[i] for i in range(len(edges)) if i not in edges_to_delete ]
                edges = edges_to_keep
                polygon_list.append(vertices)
            return polygon_list

        def refine_edges(edges, epsilon):
            indexes = []
            for i in range(len(edges)-1):
                for j in range(i+1, len(edges)):
                    if j==i:
                        pass
                    else:
                        if edges[i][0] == edges[j][0] or edges[i][1] == edges[j][1] or edges[i][0] == edges[j][1] or edges[i][1] == edges[j][0]:
                            a1, b1, c1 = get_equation(edges[i][0], edges[i][1])
                            l1 = get_edge_length(edges[i][0], edges[i][1])
                            a2, b2, c2 = get_equation(edges[j][0], edges[j][1])
                            l2 = get_edge_length(edges[i][0], edges[i][1])
                            delta = (a2-a1)**2 + (b2-b1)**2
                            if  0<delta < epsilon:
                                if l2>l1:
                                    indexes.append(j)
                                else:
                                    indexes.append(i)
            return list(edges[i] for i in set(indexes))


        edged_image = draw_contour(self.image)
        corners = detect_corners2(self.image)
        edges = get_edges(corners, edged_image
        new_edges = subdivide_edges(edges)
        ordered_vertices_2 = order_vertices_multi(new_edges, 50.5)
        vertices2 = np.array([list(vertex) for vertex in ordered_vertices_2])

        
        for V in ordered_vertices_2:
            ordered_positions = [self.map.index_to_coord(a[0], a[1]) for a in V]
            obstacle = Polygon(ordered_positions)
            obstacles.append(obstacle)
        return obstacles
