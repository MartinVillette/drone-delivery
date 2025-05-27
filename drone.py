"""
Drone Delivery Path Planning System

This module implements a 3D pathfinding system for drone delivery services using
Dijkstra's algorithm and A* search on a grid-based representation of urban terrain.
The system loads 3D city data and finds optimal flight paths while avoiding buildings.

@author Martin
@created 2023
@version 1.0.0
"""

import open3d as o3d
import numpy as np
import random, pygame
from time import time

class Drone():
    """
    Represents a delivery drone with flight altitude constraints.
    
    The drone operates at a fixed altitude and can only traverse nodes
    below its maximum flight height for safety regulations.
    """
    
    def __init__(self, altitude):
        """
        Initialize drone with specified flight altitude.
        
        Args:
            altitude (float): Maximum flight altitude in meters
        """
        self.altitude = altitude

class Node():
    """
    Represents a grid point in 3D space with connectivity information.
    
    Each node stores its 3D coordinates, grid indices, and connections
    to neighboring nodes for pathfinding algorithms.
    """
    
    def __init__(self, coords, index):
        """
        Initialize a grid node with position and index.
        
        Args:
            coords (tuple): 3D coordinates (x, y, z) in meters
            index (tuple): Grid indices (i, j) for 2D grid position
        """
        self.x, self.y, self.z = coords
        self.i, self.j = index
        self.neighbours = []
        self.explored = False  # Flag for pathfinding algorithms

    def add_neighbour(self, node):
        """
        Add bidirectional connection to another node.
        
        Args:
            node (Node): Neighboring node to connect
        """
        if node not in self.neighbours:
            self.neighbours.append(node)
    
    def suppr(self):
        """Remove this node from all neighbor connections."""
        for neighbour in self.neighbours:
            neighbour.neighbours.remove(self)
    
    def __str__(self):
        """String representation showing 2D coordinates."""
        return f'x : {self.x}, y : {self.y}'

class City():
    """
    Main class for 3D city representation and drone pathfinding.
    
    Loads 3D mesh data of buildings and terrain, creates a navigation grid,
    and implements pathfinding algorithms for drone delivery routes.
    """
    
    def __init__(self):
        """
        Initialize city with default parameters and load 3D data.
        
        Creates a 300x300 grid representation of the city and loads
        building/terrain meshes from the specified directory.
        """
        self.dimension = 300  # Grid resolution (300x300 nodes)
        self.drone = Drone(45)  # Drone flying at 45m altitude
        self.directory = 'objects/paris/'  # Path to 3D mesh files
        self.load_data()
        self.build()
        self.run()
    
    def run(self):
        """
        Execute complete pathfinding demo with random start/end points.
        
        Selects random accessible nodes, runs both Dijkstra and A* algorithms,
        and displays results in 3D visualization.
        """
        # Find random start node below drone altitude
        i = random.randint(0, len(self.nodes)-1)
        while self.nodes[i].z > self.drone.altitude:
            i = random.randint(0, len(self.nodes)-1)

        # Find random end node (different from start) below drone altitude
        j = random.randint(0, len(self.nodes)-1)
        while j == i and self.nodes[j].z > self.drone.altitude:
            j = random.randint(0, len(self.nodes)-1)
            
        self.start = self.nodes[i]
        self.end = self.nodes[j]
        self.results = []
        
        # Run both pathfinding algorithms for comparison
        self.dijkstra()
        self.a_star()
        self.show()

    def load_data(self):
        """
        Load 3D mesh data from OBJ files (made with Blender) and extract point cloud.
        
        Loads terrain and building meshes, samples points from building surfaces
        for height calculation, and prepares data for grid construction.
        """
        print('LOAD DATA...')
        t = time()
        
        # Load terrain mesh for boundary calculation
        self.terrain = o3d.io.read_triangle_mesh(self.directory + 'terrain.obj')
        
        # Load building mesh and compute normals for visualization
        self.mesh = o3d.io.read_triangle_mesh(self.directory + 'buildings.obj')
        self.mesh.compute_vertex_normals()
        
        # Sample points from building surfaces for height queries
        pcd = self.mesh.sample_points_poisson_disk(number_of_points=len(self.mesh.vertices))
        o3d.visualization.draw_geometries([self.mesh, pcd])
        
        # Convert point cloud to numpy array for efficient processing
        self.data = np.array(pcd.points)
        print(f'Loading time : {time() - t}s')

    def build(self):
        """
        Construct navigation grid from 3D city data.
        
        Creates a regular grid over the terrain bounds, calculates maximum
        building height in each cell, and establishes neighbor connections
        for pathfinding algorithms.
        """
        print('BUILD...')
        t = time()
        
        n = self.dimension
        min_coords = self.terrain.get_min_bound()
        max_coords = self.terrain.get_max_bound()
        
        # Calculate grid cell dimensions
        dx, dy = (max_coords[0] - min_coords[0]) / n, (max_coords[1] - min_coords[1]) / n
        
        grid = []
        self.nodes = []
        
        # Create grid nodes with height data
        for i in range(n):
            row = []
            for j in range(n):
                # Define cell boundaries
                min_x, max_x = min_coords[0] + i * dx, min_coords[0] + (i+1) * dx
                min_y, max_y = min_coords[1] + j * dy, min_coords[1] + (j+1) * dy
                
                # Find all building points within this grid cell
                mask = (self.data[:,0] <= max_x) & (self.data[:,0] >= min_x) & \
                       (self.data[:,1] <= max_y) & (self.data[:,1] >= min_y)
                l = [z for (x,y,z) in self.data[mask]]
                
                # Set node position at cell center with maximum building height
                x, y = (max_x + min_x) / 2, (max_y + min_y) / 2
                z = max(l) if l else 0  # Ground level if no buildings
                
                node = Node((x,y,z), (i,j))
                row.append(node)
            self.nodes += row
            grid.append(row)

        # Establish 8-directional neighbor connections
        for x in range(n):
            for y in range(n):
                node = grid[x][y]
                # Connect to all adjacent cells (including diagonals)
                for i in range(x - 1, x + 2):
                    for j in range(y - 1, y + 2):
                        if 0 <= i < n and 0 <= j < n and (i,j) != (x,y):
                            node.add_neighbour(grid[i][j])

        print(f'Building time : {time() - t}s')
        
    def distance(self, nodeA, nodeB):
        """
        Calculate Euclidean distance between two nodes (2D only).
        
        Args:
            nodeA (Node): First node
            nodeB (Node): Second node
            
        Returns:
            float: Euclidean distance in meters
        """
        return ((nodeA.x - nodeB.x)**2 + (nodeA.y - nodeB.y)**2)** 0.5

    def dijkstra(self):
        """
        Execute Dijkstra's algorithm and store results.
        
        Finds shortest path from start to end node, measures execution time,
        and stores path data for visualization with orange color coding.
        """
        print('Dijkstra')
        t = time()
        
        distance = self.dijkstraAlgo(self.start, self.end)
        total = distance[self.end]['distance']
        
        # Reconstruct path by backtracking from end to start
        node = self.end
        path = [node]
        while node != self.start:
            node = distance[node]['source']
            path.append(node)
            
        print(f'process time : {time() - t}s, distance : {int(total)}m')
        
        # Store results with orange color for visualization
        color = [x/255 for x in [209,96,20]]
        self.results.append({
            'algo': 'Dijkstra',
            'distance': int(total),
            'color': color,
            'path': path,
            'altitude': self.drone.altitude
        })

    def dijkstraAlgo(self, start, end):
        """
        Dijkstra's shortest path algorithm implementation.
        
        Args:
            start (Node): Starting node
            end (Node): Target node
            
        Returns:
            dict: Distance and source information for each explored node
        """
        queue = [start]
        distance = {start: {'distance': 0, 'source': None}}
        
        while queue:
            current = queue.pop(0)
            if current == end:
                return distance
                
            for neighbour in current.neighbours:
                # Only consider nodes below drone altitude for safety
                if neighbour.z < self.drone.altitude:
                    if not neighbour.explored:
                        queue.append(neighbour)
                    neighbour.explored = True
                    
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    
                    # Update if shorter path found
                    if (neighbour not in distance or new_dist < distance[neighbour]['distance']):
                        distance[neighbour] = {'distance': new_dist, 'source': current}
        return distance
    
    def a_star(self):
        """
        Execute A* algorithm and store results.
        
        Finds shortest path using A* heuristic search, measures execution time,
        and stores path data for visualization with blue color coding.
        """
        print('A*')
        t = time()
        
        distance = self.a_star_algo(self.start, self.end)
        
        # Reconstruct path by backtracking
        path = []
        node = self.end
        total = distance[self.end]['distance']
        path = [node]
        while node != self.start:
            node = distance[node]['source']
            path.append(node)
            
        print(f'process time : {time() - t}s, distance : {int(total)}m')
        
        # Store results with blue color for visualization
        color = [x/255 for x in [68,204,255]]
        self.results.append({
            'algo': 'A*',
            'distance': int(total),
            'color': color,
            'path': path,
            'altitude': self.drone.altitude
        })

    def a_star_algo(self, start, end):
        """
        A* pathfinding algorithm with Euclidean distance heuristic.
        
        Args:
            start (Node): Starting node
            end (Node): Target node
            
        Returns:
            dict: Distance and source information for each explored node
        """
        queue = [{'node': start, 'priority': 0}]
        distance = {start: {'distance': 0, 'source': None}}
        
        while queue:
            current = queue.pop(0)['node']
            if current == end:
                return distance
                
            for neighbour in current.neighbours:
                # Only consider nodes below drone altitude
                if neighbour.z < self.drone.altitude:
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    
                    # Update if shorter path found
                    if neighbour not in distance or new_dist < distance[neighbour]['distance']:
                        distance[neighbour] = {'distance': new_dist, 'source': current}
                        
                        # A* priority = distance + heuristic (straight-line distance to goal)
                        priority = new_dist + self.distance(neighbour, end)
                        queue.append({'node': neighbour, 'priority': priority})
                        
                        # Keep queue sorted by priority for efficient processing
                        queue.sort(key=lambda x: x['priority'])
        return distance
    
    def show_2d(self):
        """
        Display 2D top-down view of paths using Pygame.
        
        Shows calculated paths overlaid on city map background for
        quick visual comparison of algorithm results.
        """
        dimensions = 700
        n = self.dimension
        background = pygame.image.load(self.directory + 'overlay.png')

        screen = pygame.display.set_mode((dimensions, dimensions))
        background = pygame.transform.scale(background, (dimensions, dimensions))
        screen.blit(background, (0, 0))

        # Scale grid coordinates to screen pixels
        dx, dy = (1 / n) * dimensions, (1 / n) * dimensions

        # Draw all calculated paths
        for result in self.results:
            for k in range(len(result['path'])-1):
                pointA = result['path'][k]
                pointB = result['path'][k+1]
                
                # Convert grid coordinates to screen coordinates
                xA, yA = pointA.i*dx, (self.dimension - pointA.j)*dy
                xB, yB = pointB.i*dx, (self.dimension - pointB.j)*dy
                
                # Draw path segment
                pygame.draw.line(screen, [x*255 for x in result['color']], (xA,yA), (xB,yB), width=3)
        pygame.display.flip()

    def show(self):
        """
        Display 3D visualization of city and calculated paths.
        
        Creates an Open3D visualization showing the terrain, buildings,
        and flight paths calculated by different algorithms at drone altitude.
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        # Visualize all calculated paths
        for result in self.results:
            colors = [result['color'] for _ in range(len(result['path'])-1)]
            
            # Create path points at drone altitude for clear visualization
            points = [(node.x, node.y, result['altitude']) for node in result['path']]
            lines = [(i, i+1) for i in range(len(result['path'])-1)]
            
            # Create line set for path visualization
            line_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
            )
            line_set.colors = o3d.utility.Vector3dVector(colors)
            vis.add_geometry(line_set)
            print(f'{result["algo"]} : {result["distance"]}m')

        # Add terrain with gray coloring
        self.terrain.paint_uniform_color([.4, .4, .4]) 
        vis.add_geometry(self.terrain)

        # Visualize grid nodes (optional - currently commented)
        points = [(node.x, node.y, node.z) for node in self.nodes]
        pcd = o3d.geometry.PointCloud(
            points=o3d.utility.Vector3dVector(points)
        )

        # Add buildings with light gray coloring
        self.mesh.paint_uniform_color([.7, .7, .7])
        vis.add_geometry(self.mesh)
        
        # Set dark background for better contrast
        opt = vis.get_render_option()
        opt.background_color = np.array([.1, .1, .1])
        
        vis.run()
        vis.destroy_window()


# Entry point - create and initialize city pathfinding system
if __name__ == "__main__":
    # Create city instance and run pathfinding demo
    city = City()