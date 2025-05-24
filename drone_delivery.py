import open3d as o3d
import numpy as np
import pygame, itertools, os
from time import time

class Drone():
    def __init__(self, altitude):
        self.altitude = altitude #altitude de vole du drone

class Node(): #Point de l'espace
    def __init__(self, coords, index):
        self.x, self.y, self.z = coords #coordonées de l'espaces dans le modèle 3D
        self.i, self.j = index #index dans la matrice de points
        self.neighbours = [] #liste des points environnents
        self.explored = False #est-ce que le point a été exploré ?

    def add_neighbour(self, node): #ajoute un point aux voisins
        if node not in self.neighbours:
            self.neighbours.append(node)

class City():
    def __init__(self):
        self.dimension = 300 #échantillonnage / nombre de points selon x (et de selon y)
        self.screen_dimension = 700 #dimensions de l'écran

        self.drone = Drone(2)
        self.directory = 'objects/paris_2/' #dossier contenant les données 3D
        self.builded = False #est-ce que la ville a été construite ? (données récupérées etc.)
        self.load_data() #charge les données
        self.build() #construit la ville / relie les points entre eux
        self.run() #lance le programme

    def load_data(self):
        print('LOAD DATA...')
        t = time()
        self.terrain = o3d.io.read_triangle_mesh(self.directory + 'terrain.obj') #terrain 3d
        self.mesh = o3d.io.read_triangle_mesh(self.directory + 'buildings.obj') #batiments 3d
        self.mesh.compute_vertex_normals()
        pcd = self.mesh.sample_points_poisson_disk(number_of_points=len(self.mesh.vertices)) #partitionne les surfaces 3d en points
        # o3d.visualization.draw_geometries([self.mesh,pcd])
        self.data = np.array(pcd.points) #matrice de points des batiments
        print(f'Loading time : {time() - t}s')

    def build(self):
        print('BUILD...')
        t = time()
        n = self.dimension
        # on récupère les coordonnées extrèmes dans lequel on va travailler / cadre de l'étude en d'espace
        min_coords = self.terrain.get_min_bound() 
        max_coords = self.terrain.get_max_bound()
        # taille d'un point dans l'espace (selon x et y)
        dx, dy = (max_coords[0] - min_coords[0]) / n, (max_coords[1] - min_coords[1]) / n
        grid = [] #matrice des points
        self.nodes = [] #liste des points

        for i in range(n):
            row = [] #la ième rangé
            for j in range(n):
                #on va récupérer tout les points contenus de carré
                min_x,max_x = min_coords[0] + i * dx,  min_coords[0] + (i+1) * dx 
                min_y,max_y = min_coords[1] + j * dy,  min_coords[1] + (j+1) * dy
                mask = (self.data[:,0] <= max_x) & (self.data[:,0] >= min_x) & (self.data[:,1] <= max_y) & (self.data[:,1] >= min_y)
                l = [z for (x,y,z) in self.data[mask]] #liste des altitudes des points contenu dans le carré
                x,y = (max_x + min_x)/2, (max_y + min_y)/2 #on positionne les coordonnées du carré en son centre
                z = max(l) if l else 0
                node = Node((x,y,z),(i,j)) #on crée le point
                row.append(node)
            self.nodes += row
            grid.append(row)

        #on relie tout les points entres eux
        for x in range(n):
            for y in range(n):
                node = grid[x][y]
                for i in range(x - 1, x + 2):
                    for j in range(y - 1, y + 2):
                        if 0 <= i < n and 0 <= j < n and (i,j) != (x,y):
                            node.add_neighbour(grid[i][j])
                        
        self.builded = True
        print(f'Building time : {time() - t}s')
    
    def run(self):
        if self.builded: #si la ville est bien construite (prévention)
            self.start = None #on démarre le programme
            self.results = [] #initialise la liste qui va contenir les informations de plus cour chemin
            self.points = [] #initialise la liste des points que l'on va livrer
            self.number_points = None #nombre de points que l'on va livrer
            while not self.number_points: #on demande le nombre de points en une livraison
                try:
                    self.number_points = int(input('Number of points : '))
                except:
                    print('Enter a valid number')            
            self.select_coordinates() #selectionne les points que l'on va livrer sur la carte et on attend qu'on en ai choisi le bon nombre
            # self.delivery()
            # self.show_2d()
            # self.show()
    
    
    def select_coordinates(self):
        #on créer la carte 2d de la ville
        dimensions = self.screen_dimension 
        background = pygame.image.load(self.directory + 'overlay.png')

        self.screen = pygame.display.set_mode((dimensions,dimensions))
        background = pygame.transform.scale(background, (dimensions,dimensions))
        self.screen.blit(background, (0,0))

        print(f'SELECT {self.number_points} POINTS')
        pygame.display.flip()

    def click(self, pos):
        if len(self.points) < self.number_points: #tant qu'on a pas sélectionner le bon nombre de points de livraison
            # on récupère le point sur lequel on a cliqué sur l'écran
            dimensions = self.screen_dimension
            n = self.dimension
            dx, dy = (1 / n) * dimensions, (1 / n) * dimensions
            x,y = pos
            i,j = x // dx, y // dy
            index = 0
            found = False
            while not found and index < len(self.nodes): #tant que le point choisi n'est pas valide
                node = self.nodes[index] #récupère le point considéré
                if (node.i, node.j) == (i,(self.dimension - j)) and node.z < self.drone.altitude: #si le point est dans la zone cliqué et que l'on peut livrer à ce point
                    if not self.start: #si on a pas encore choisi le point
                        self.start = node #le point devient l'entrepôt : lieu de départ de tout les colis
                        color = (200,50,0) #il est représenté en rouge
                    else:
                        self.points.append(node) #sinon, il s'agit d'un point de livraison
                        color = (0,50,200) #bleu
                    found = True #le point est possible
                    pygame.draw.circle(self.screen,color,(i*dx,j*dy), 5) #on le représente par un cercle sur la carte
                    pygame.display.flip()
                index += 1 #sinon, on considère le point suivant
        
        if len(self.points) == self.number_points: #si on a choisi tout les lieux de livraisons
            self.delivery() #on peut lancer le programme de livraison / plus court chemin
        
    def reset(self):
        pygame.display.flip()
        self.run()
    
    def screenshot(self):
        if self.builded:
            print('Screenshot')
            path = 'screenshots/'
            pygame.image.save(self.screen, f'{path}/image_{len(os.listdir(path))}.png')
        
    def distance(self, nodeA, nodeB):
        return ((nodeA.x - nodeB.x)**2 + (nodeA.y - nodeB.y)**2)** 0.5

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

    def delivery(self):
        print('DELIVERY...')            
        self.screenshot()       
        temp_permutations = list(itertools.permutations(self.points)) #on considère toute les permutations possible des livraisons / ordre de livraison
        temp_permutations = [[self.start] + list(perm) + [self.start] for perm in temp_permutations] #on y ajoute l'entrepôt au début et à la fin (on part de l'entrepôt, on arrive à l'entrepôt)
        #on ne considère que la moitié des permutations : qu'elles se fassent dans un sens ou dans l'autre, cela n'a pas d'importance
        permutations = []
        for perm in temp_permutations:
            if not perm[::-1] in permutations:
                permutations.append(perm)
        #on place l'altitude du drone à 2m : cela représente une voiture
        self.drone.altitude = 2
        self.delivery_truck(permutations)
        #on place l'altitude du drone à 45m : cela représente le drone
        self.drone.altitude = 45
        self.delivery_drone(self.points)
        self.show_2d()
        self.screenshot()    
        # self.show()


    def delivery_truck(self, permutations):
        result = []
        for perm in permutations:
            path = [self.start]
            total = 0
            i = len(perm) - 2
            path_possible = True
            while i >= 0 and path_possible:
                start, end = perm[i], perm[i+1]
                try:
                    distance = self.a_star_algo(start, end)
                except:
                    path_possible = False
                else:
                    node = end
                    if not end in distance:
                        path_possible = False
                    else:
                        total += distance[end]['distance']
                        while node != start:
                            node = distance[node]['source']
                            path.append(node)
                        i -= 1
            if path_possible:
                h = {'permutations':perm, 'path':path, 'distance':int(total)}
                result.append(h)

        if result:
            best_path = min(result, key=lambda x:x['distance'])
            color = [x/255 for x in [68,204,255]]
            self.results.append({'algo':'Voiture','distance':int(best_path['distance']),'color':color,'path':best_path['path'],'altitude':self.drone.altitude})
        else:
            print('NO PATH : Car')


    def delivery_drone(self, points):
        final_path = []
        total = 0
        for point in points:
            distance = self.a_star_algo(self.start, point)
            node = point
            total += 2 * distance[point]['distance']
            path = [node]
            while node != self.start:
                node = distance[node]['source']
                path.append(node)
            final_path += path[::-1] + path[1:-1]

        color = [x/255 for x in [209,96,20]]
        self.results.append({'algo':'Drone','distance':int(total),'color':color,'path':final_path,'altitude':self.drone.altitude})


    def show_2d(self):
        dimensions = self.screen_dimension
        n = self.dimension
        dx, dy = (1 / n) * dimensions, (1 / n) * dimensions

        for result in self.results:
            print(f'{result["algo"]} : {result["distance"]}m')
            for k in range(len(result['path'])-1):
                pointA = result['path'][k]
                pointB = result['path'][k+1]
                xA,yA = pointA.i*dx, (self.dimension - pointA.j)*dy
                xB,yB = pointB.i*dx, (self.dimension - pointB.j)*dy
                pygame.draw.line(self.screen,[x*255 for x in result['color']],(xA,yA),(xB,yB), width=4)
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
        
        # terrain = o3d.io.read_triangle_mesh(self.directory + 'terrain.obj')
        self.terrain.paint_uniform_color([.4,.4,.4]) 
        vis.add_geometry(self.terrain)

        points = [(node.x,node.y,node.z) for node in self.nodes]
        
        self.mesh.paint_uniform_color([.7,.7,.7])
        vis.add_geometry(self.mesh) 
        opt = vis.get_render_option()
        opt.background_color = np.array([.1,.1,.1])
        vis.run()
        vis.destroy_window()

city = City()
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            city.click(pos)

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                city.reset()

            if event.key == pygame.K_s:
                city.screenshot()
