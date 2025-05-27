"""
Interactive Drone vs Truck Delivery Optimization System

This module provides an interactive interface for comparing delivery efficiency 
between drones and ground vehicles. Users can select delivery points on a city map,
and the system calculates optimal routes using A* pathfinding for both transport modes.

The system demonstrates:
- Interactive point selection on 2D city maps
- Multi-point delivery route optimization using permutations
- Comparative analysis between aerial (drone) and ground (truck) transport
- Real-time visualization of calculated routes

@author Martin
@created 2023
@version 1.0.0
"""

import open3d as o3d
import numpy as np
import pygame, itertools, os
from time import time

class Drone():
    """
    Represents a delivery vehicle with altitude-based movement constraints.
    
    Can represent either a drone (high altitude) or ground vehicle (low altitude)
    depending on the altitude setting for route planning purposes.
    """
    
    def __init__(self, altitude):
        """
        Initialize delivery vehicle with maximum operating altitude.
        
        Args:
            altitude (float): Maximum operating altitude in meters
                             (2m for trucks, 45m for drones)
        """
        self.altitude = altitude

class Node():
    """
    Represents a navigable point in 3D city space.
    
    Each node contains spatial coordinates, grid position, and connectivity
    information for pathfinding algorithms.
    """
    
    def __init__(self, coords, index):
        """
        Initialize a navigation node with position data.
        
        Args:
            coords (tuple): 3D coordinates (x, y, z) in meters
            index (tuple): Grid indices (i, j) for 2D representation
        """
        self.x, self.y, self.z = coords  # 3D spatial coordinates
        self.i, self.j = index  # Grid matrix indices
        self.neighbours = []  # Connected adjacent nodes
        self.explored = False  # Pathfinding exploration flag

    def add_neighbour(self, node):
        """
        Establish bidirectional connection with another node.
        
        Args:
            node (Node): Adjacent node to connect
        """
        if node not in self.neighbours:
            self.neighbours.append(node)

class City():
    """
    Main delivery optimization system with interactive route planning.
    
    Provides interactive map interface for selecting delivery points and 
    automatically calculates optimal routes for both drone and truck delivery.
    Compares efficiency between transport modes.
    """
    
    def __init__(self):
        """
        Initialize the delivery system and start interactive interface.
        
        Sets up city grid, loads 3D data, and launches the interactive
        point selection interface for delivery route planning.
        """
        self.dimension = 300  # Grid resolution (300x300 nodes)
        self.screen_dimension = 700  # Display window size in pixels
        
        self.drone = Drone(2)  # Initialize as ground vehicle (2m altitude)
        self.directory = 'objects/paris/'  # 3D data directory
        self.builded = False  # City construction status flag
        
        # Initialize system components
        self.load_data()
        self.build()
        self.run()

    def load_data(self):
        """
        Load 3D city mesh data from OBJ files.
        
        Loads terrain and building geometry, samples building surfaces
        for height data extraction used in grid construction.
        """
        print('LOAD DATA...')
        t = time()
        
        # Load 3D terrain mesh for boundary calculation
        self.terrain = o3d.io.read_triangle_mesh(self.directory + 'terrain.obj')
        
        # Load building mesh and prepare for height sampling
        self.mesh = o3d.io.read_triangle_mesh(self.directory + 'buildings.obj')
        self.mesh.compute_vertex_normals()
        
        # Sample building surfaces to extract height data for each grid cell
        pcd = self.mesh.sample_points_poisson_disk(number_of_points=len(self.mesh.vertices))
        self.data = np.array(pcd.points)  # Convert to numpy for efficient processing
        
        print(f'Loading time : {time() - t}s')

    def build(self):
        """
        Construct navigation grid from 3D city data.
        
        Creates uniform grid over city bounds, calculates maximum building
        height per cell, and establishes 8-directional connectivity between nodes.
        """
        print('BUILD...')
        t = time()
        
        n = self.dimension
        # Define working area boundaries from terrain data
        min_coords = self.terrain.get_min_bound() 
        max_coords = self.terrain.get_max_bound()
        
        # Calculate grid cell dimensions
        dx, dy = (max_coords[0] - min_coords[0]) / n, (max_coords[1] - min_coords[1]) / n
        
        grid = []  # 2D grid matrix for neighbor lookup
        self.nodes = []  # Flat list of all nodes

        # Generate grid nodes with building height data
        for i in range(n):
            row = []
            for j in range(n):
                # Define current cell boundaries
                min_x, max_x = min_coords[0] + i * dx, min_coords[0] + (i+1) * dx 
                min_y, max_y = min_coords[1] + j * dy, min_coords[1] + (j+1) * dy
                
                # Find all building points within current cell
                mask = (self.data[:,0] <= max_x) & (self.data[:,0] >= min_x) & \
                       (self.data[:,1] <= max_y) & (self.data[:,1] >= min_y)
                height_list = [z for (x,y,z) in self.data[mask]]
                
                # Position node at cell center with maximum building height
                x, y = (max_x + min_x)/2, (max_y + min_y)/2
                z = max(height_list) if height_list else 0  # Ground level if no buildings
                
                node = Node((x,y,z), (i,j))
                row.append(node)
            
            self.nodes += row
            grid.append(row)

        # Establish 8-directional connectivity (including diagonals)
        for x in range(n):
            for y in range(n):
                node = grid[x][y]
                # Connect to all adjacent cells
                for i in range(x - 1, x + 2):
                    for j in range(y - 1, y + 2):
                        if 0 <= i < n and 0 <= j < n and (i,j) != (x,y):
                            node.add_neighbour(grid[i][j])
                        
        self.builded = True
        print(f'Building time : {time() - t}s')
    
    def run(self):
        """
        Initialize interactive delivery point selection interface.
        
        Prompts user for number of delivery points and starts interactive
        map interface for point selection.
        """
        if self.builded:
            self.start = None  # Depot/warehouse location
            self.results = []  # Route calculation results storage
            self.points = []  # Selected delivery locations
            self.number_points = None
            
            # Get number of delivery points from user
            while not self.number_points:
                try:
                    self.number_points = int(input('Number of delivery points: '))
                except:
                    print('Enter a valid number')
                    
            self.select_coordinates()
    
    def select_coordinates(self):
        """
        Display interactive map for delivery point selection.
        
        Shows 2D city overlay and waits for user to click delivery locations.
        First click sets depot, subsequent clicks set delivery points.
        """
        dimensions = self.screen_dimension 
        background = pygame.image.load(self.directory + 'overlay.png')

        # Create display window with city overlay
        self.screen = pygame.display.set_mode((dimensions, dimensions))
        background = pygame.transform.scale(background, (dimensions, dimensions))
        self.screen.blit(background, (0, 0))

        print(f'SELECT {self.number_points} DELIVERY POINTS')
        print('First click: Depot (red), Next clicks: Delivery points (blue)')
        pygame.display.flip()

    def click(self, pos):
        """
        Handle mouse click events for point selection.
        
        Converts screen coordinates to grid coordinates and validates
        that selected points are accessible (below vehicle altitude).
        
        Args:
            pos (tuple): Mouse click position (x, y) in screen pixels
        """
        if len(self.points) < self.number_points:
            # Convert screen coordinates to grid indices
            dimensions = self.screen_dimension
            n = self.dimension
            dx, dy = (1 / n) * dimensions, (1 / n) * dimensions
            x, y = pos
            i, j = x // dx, y // dy
            
            # Find corresponding node in grid
            index = 0
            found = False
            while not found and index < len(self.nodes):
                node = self.nodes[index]
                # Check if click matches node position and altitude is accessible
                if (node.i, node.j) == (i, (self.dimension - j)) and node.z < self.drone.altitude:
                    if not self.start:
                        # First click sets depot location
                        self.start = node
                        color = (200, 50, 0)  # Red for depot
                    else:
                        # Subsequent clicks add delivery points
                        self.points.append(node)
                        color = (0, 50, 200)  # Blue for delivery points
                    
                    found = True
                    # Visualize selected point on map
                    pygame.draw.circle(self.screen, color, (i*dx, j*dy), 5)
                    pygame.display.flip()
                index += 1
        
        # Start route optimization when all points selected
        if len(self.points) == self.number_points:
            self.delivery()
        
    def reset(self):
        """Reset interface and restart point selection process."""
        pygame.display.flip()
        self.run()
    
    def screenshot(self):
        """Save current map display as PNG screenshot."""
        if self.builded:
            print('Screenshot saved')
            path = 'screenshots/'
            pygame.image.save(self.screen, f'{path}/image_{len(os.listdir(path))}.png')
        
    def distance(self, nodeA, nodeB):
        """
        Calculate Euclidean distance between two nodes.
        
        Args:
            nodeA (Node): First node
            nodeB (Node): Second node
            
        Returns:
            float: Euclidean distance in meters
        """
        return ((nodeA.x - nodeB.x)**2 + (nodeA.y - nodeB.y)**2)** 0.5

    def a_star_algo(self, start, end):
        """
        A* pathfinding algorithm with Euclidean distance heuristic.
        
        Args:
            start (Node): Starting node
            end (Node): Target node
            
        Returns:
            dict: Distance and source information for path reconstruction
        """
        queue = [{'node': start, 'priority': 0}]
        distance = {start: {'distance': 0, 'source': None}}
        
        while queue:
            current = queue.pop(0)['node']
            if current == end:
                return distance
                
            for neighbour in current.neighbours:
                # Only traverse nodes below vehicle altitude
                if neighbour.z < self.drone.altitude:
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    
                    if neighbour not in distance or new_dist < distance[neighbour]['distance']:
                        distance[neighbour] = {'distance': new_dist, 'source': current}
                        # A* heuristic: actual distance + straight-line distance to goal
                        priority = new_dist + self.distance(neighbour, end)
                        queue.append({'node': neighbour, 'priority': priority})
                        queue.sort(key=lambda x: x['priority'])
        return distance

    def delivery(self):
        """
        Execute delivery route optimization for both transport modes.
        
        Compares truck delivery (visiting all points in optimal order) with
        drone delivery (individual trips from depot to each point and back).
        """
        print('CALCULATING OPTIMAL DELIVERY ROUTES...')            
        self.screenshot()
        
        # Generate all possible delivery sequences for truck optimization
        temp_permutations = list(itertools.permutations(self.points))
        # Add depot at start and end of each route (round trip)
        temp_permutations = [[self.start] + list(perm) + [self.start] for perm in temp_permutations]
        
        # Remove duplicate routes (same route in reverse order)
        permutations = []
        for perm in temp_permutations:
            if not perm[::-1] in permutations:
                permutations.append(perm)
        
        # Calculate optimal truck route (ground level, 2m altitude)
        self.drone.altitude = 2
        self.delivery_truck(permutations)
        
        # Calculate drone delivery routes (aerial, 45m altitude)
        self.drone.altitude = 45
        self.delivery_drone(self.points)
        
        # Display results
        self.show_2d()
        self.screenshot()

    def delivery_truck(self, permutations):
        """
        Calculate optimal truck delivery route using permutation analysis.
        
        Tests all possible delivery sequences to find the shortest total route
        that visits all points exactly once before returning to depot.
        
        Args:
            permutations (list): All possible delivery sequences to evaluate
        """
        result = []
        
        for perm in permutations:
            path = [self.start]
            total = 0
            i = len(perm) - 2
            path_possible = True
            
            # Calculate route distance for current permutation
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
                        # Reconstruct path segment
                        while node != start:
                            node = distance[node]['source']
                            path.append(node)
                        i -= 1
            
            if path_possible:
                result.append({
                    'permutations': perm, 
                    'path': path, 
                    'distance': int(total)
                })

        if result:
            # Select shortest route from all valid permutations
            best_path = min(result, key=lambda x: x['distance'])
            color = [x/255 for x in [68, 204, 255]]  # Blue for truck routes
            self.results.append({
                'algo': 'Truck',
                'distance': int(best_path['distance']),
                'color': color,
                'path': best_path['path'],
                'altitude': self.drone.altitude
            })
        else:
            print('NO VALID PATH FOUND: Truck delivery impossible')

    def delivery_drone(self, points):
        """
        Calculate drone delivery routes using hub-and-spoke model.
        
        Each delivery point is visited individually with return trips to depot.
        Total distance includes round trips to each delivery location.
        
        Args:
            points (list): Delivery locations to visit
        """
        final_path = []
        total = 0
        
        # Calculate round trip to each delivery point
        for point in points:
            distance = self.a_star_algo(self.start, point)
            node = point
            # Double distance for round trip (to point and back)
            total += 2 * distance[point]['distance']
            
            # Reconstruct path from depot to delivery point
            path = [node]
            while node != self.start:
                node = distance[node]['source']
                path.append(node)
            
            # Add outbound and return journey to final path
            final_path += path[::-1] + path[1:-1]

        color = [x/255 for x in [209, 96, 20]]  # Orange for drone routes
        self.results.append({
            'algo': 'Drone',
            'distance': int(total),
            'color': color,
            'path': final_path,
            'altitude': self.drone.altitude
        })

    def show_2d(self):
        """
        Display calculated routes on 2D map interface.
        
        Overlays truck and drone routes on city map with different colors
        and prints distance comparison results.
        """
        dimensions = self.screen_dimension
        n = self.dimension
        dx, dy = (1 / n) * dimensions, (1 / n) * dimensions

        # Draw all calculated routes
        for result in self.results:
            print(f'{result["algo"]} total distance: {result["distance"]}m')
            
            # Draw route as connected line segments
            for k in range(len(result['path'])-1):
                pointA = result['path'][k]
                pointB = result['path'][k+1]
                
                # Convert grid coordinates to screen coordinates
                xA, yA = pointA.i*dx, (self.dimension - pointA.j)*dy
                xB, yB = pointB.i*dx, (self.dimension - pointB.j)*dy
                
                pygame.draw.line(
                    self.screen, 
                    [x*255 for x in result['color']], 
                    (xA, yA), (xB, yB), 
                    width=4
                )
        pygame.display.flip()

    def show(self):
        """
        Display 3D visualization of calculated routes and city model.
        
        Creates Open3D visualization showing terrain, buildings, and
        calculated delivery routes at their respective altitudes.
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        # Visualize all calculated routes in 3D
        for result in self.results:
            colors = [result['color'] for _ in range(len(result['path'])-1)]
            # Position routes at transport altitude for clear separation
            points = [(node.x, node.y, result['altitude']) for node in result['path']]
            lines = [(i, i+1) for i in range(len(result['path'])-1)]
            
            line_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
            )
            line_set.colors = o3d.utility.Vector3dVector(colors)
            vis.add_geometry(line_set)
            print(f'{result["algo"]} total distance: {result["distance"]}m')
        
        # Add terrain with gray coloring
        self.terrain.paint_uniform_color([.4, .4, .4]) 
        vis.add_geometry(self.terrain)

        # Add buildings with light gray coloring
        self.mesh.paint_uniform_color([.7, .7, .7])
        vis.add_geometry(self.mesh)
        
        # Set dark background for better contrast
        opt = vis.get_render_option()
        opt.background_color = np.array([.1, .1, .1])
        
        vis.run()
        vis.destroy_window()


# Application entry point and event handling
city = City()
running = True

print("\nControls:")
print("- Click to select depot (first) and delivery points")
print("- Press 'R' to reset and start over")
print("- Press 'S' to save screenshot")

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