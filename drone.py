import open3d as o3d
import numpy as np
import random, pygame
from time import time

class Drone():
    def __init__(self, altitude):
        self.altitude = altitude

class Node():
    def __init__(self, coords, index):
        self.x, self.y, self.z = coords
        self.i, self.j = index
        self.neighbours = []
        self.explored = False

    def add_neighbour(self, node):
        if node not in self.neighbours:
            self.neighbours.append(node)
    
    def suppr(self):
        for neighbour in self.neighbours:
            neighbour.neighbours.remove(self)
    
    def __str__(self):
        return f'x : {self.x}, y : {self.y}'

class City():
    def __init__(self):
        self.dimension = 300
        self.drone = Drone(45)
        self.directory = 'objects/paris_2/'
        self.load_data()
        self.build()
        # self.run()
    
    def run(self):
        i = random.randint(0,len(self.nodes)-1)
        while self.nodes[i].z > self.drone.altitude:
            i = random.randint(0,len(self.nodes)-1)

        j = random.randint(0,len(self.nodes)-1)
        while j == i and self.nodes[j].z > self.drone.altitude:
            j = random.randint(0,len(self.nodes)-1)
        self.start = self.nodes[i]
        self.end = self.nodes[j]
        self.results = []
        self.dijkstra()
        self.a_star()
        # self.show_2d()
        self.show()

    def load_data(self):
        print('LOAD DATA...')
        t = time()
        self.terrain = o3d.io.read_triangle_mesh(self.directory + 'terrain.obj')
        self.mesh = o3d.io.read_triangle_mesh(self.directory + 'buildings.obj')
        self.mesh.compute_vertex_normals()
        pcd = self.mesh.sample_points_poisson_disk(number_of_points=len(self.mesh.vertices))
        o3d.visualization.draw_geometries([self.mesh,pcd])
        self.data = np.array(pcd.points)
        print(f'Loading time : {time() - t}s')

    def build(self):
        print('BUILD...')
        t = time()
        n = self.dimension
        min_coords = self.terrain.get_min_bound()
        max_coords = self.terrain.get_max_bound()
        dx, dy = (max_coords[0] - min_coords[0]) / n, (max_coords[1] - min_coords[1]) / n
        grid = []
        self.nodes = []
        for i in range(n):
            row = []
            for j in range(n):
                min_x,max_x = min_coords[0] + i * dx,  min_coords[0] + (i+1) * dx
                min_y,max_y = min_coords[1] + j * dy,  min_coords[1] + (j+1) * dy
                mask = (self.data[:,0] <= max_x) & (self.data[:,0] >= min_x) & (self.data[:,1] <= max_y) & (self.data[:,1] >= min_y)
                l = [z for (x,y,z) in self.data[mask]]
                x,y = (max_x + min_x) / 2, (max_y + min_y) / 2
                z = max(l) if l else 0
                node = Node((x,y,z),(i,j))
                row.append(node)
            self.nodes += row
            grid.append(row)

        for x in range(n):
            for y in range(n):
                node = grid[x][y]
                for i in range(x - 1, x + 2):
                    for j in range(y - 1, y + 2):
                        if 0 <= i < n and 0 <= j < n and (i,j) != (x,y):
                            node.add_neighbour(grid[i][j])

        print(f'Building time : {time() - t}s')
        
    def distance(self, nodeA, nodeB):
        return ((nodeA.x - nodeB.x)**2 + (nodeA.y - nodeB.y)**2)** 0.5

    def dijkstra(self):
        print('Dijkstra')
        t = time()
        distance = self.dijkstraAlgo(self.start, self.end)
        total = distance[self.end]['distance']
        node = self.end
        path = [node]
        while node != self.start:
            node = distance[node]['source']
            path.append(node)
        print(f'process time : {time() - t}s, distance : {int(total)}m')
        color = [x/255 for x in [209,96,20]]
        self.results.append({'algo':'A*','distance':int(total),'color':color,'path':path, 'altitude':self.drone.altitude})

    def dijkstraAlgo(self, start, end):
        queue = [start]
        distance = {start:{'distance':0,'source':None}}
        while queue:
            current = queue.pop(0)
            if current == end:
                return distance
            for neighbour in current.neighbours:
                if neighbour.z < self.drone.altitude:
                    if not neighbour.explored:
                        queue.append(neighbour)
                    neighbour.explored = True
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    if (neighbour not in distance or new_dist < distance[neighbour]['distance']):
                        distance[neighbour] = {'distance':new_dist,'source':current}
        return distance
    
    def a_star(self):
        print('A*')
        t = time()
        distance = self.a_star_algo(self.start, self.end)
        path = []
        node = self.end
        total = distance[self.end]['distance']
        path = [node]
        while node != self.start:
            node = distance[node]['source']
            path.append(node)
        print(f'process time : {time() - t}s, distance : {int(total)}m')
        color = [x/255 for x in [68,204,255]]
        self.results.append({'algo':'A*','distance':int(total),'color':color,'path':path, 'altitude':self.drone.altitude})

    def a_star_algo(self, start, end):
        queue = [{'node':start,'priority':0}]
        distance = {start:{'distance':0,'source':None}}
        while queue:
            current = queue.pop(0)['node']
            if current == end:
                return distance
            for neighbour in current.neighbours:
                if neighbour.z < self.drone.altitude:
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    if neighbour not in distance or new_dist < distance[neighbour]['distance']:
                        distance[neighbour] = {'distance':new_dist,'source':current}
                        priority = new_dist + self.distance(neighbour, end)
                        queue.append({'node':neighbour,'priority':priority})
                        queue.sort(key=lambda x:x['priority'])
        return distance
    
    
    def show_2d(self):
        dimensions = 700
        n = self.dimension
        background = pygame.image.load(self.directory + 'overlay.png')

        screen = pygame.display.set_mode((dimensions,dimensions))
        background = pygame.transform.scale(background, (dimensions,dimensions))
        screen.blit(background, (0,0))

        dx, dy = (1 / n) * dimensions, (1 / n) * dimensions

        for result in self.results:
            for k in range(len(result['path'])-1):
                pointA = result['path'][k]
                pointB = result['path'][k+1]
                xA,yA = pointA.i*dx, (self.dimension - pointA.j)*dy
                xB,yB = pointB.i*dx, (self.dimension - pointB.j)*dy
                # pygame.draw.circle(screen, [x*255 for x in result['color']],(xA,yA), 3)
                pygame.draw.line(screen,[x*255 for x in result['color']],(xA,yA),(xB,yB), width=3)
        pygame.display.flip()

    def show(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        for result in self.results:
            colors = [result['color'] for _ in range(len(result['path'])-1)]
            points = [(node.x,node.y,result['altitude']) for node in result['path']]
            lines = [(i,i+1) for i in range(len(result['path'])-1)]
            line_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
                )
            line_set.colors = o3d.utility.Vector3dVector(colors)
            vis.add_geometry(line_set)
            print(f'{result["algo"]} : {result["distance"]}m')

        self.terrain.paint_uniform_color([.4,.4,.4]) 
        vis.add_geometry(self.terrain)


        points = [(node.x,node.y,node.z) for node in self.nodes]
        pcd = o3d.geometry.PointCloud(
            points=o3d.utility.Vector3dVector(points)
        )

        self.mesh.paint_uniform_color([.7,.7,.7])
        vis.add_geometry(self.mesh)
        opt = vis.get_render_option()
        opt.background_color = np.array([.1,.1,.1])
        vis.run()
        vis.destroy_window()


city = City()