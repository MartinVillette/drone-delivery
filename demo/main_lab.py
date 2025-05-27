"""
Interactive Pathfinding Algorithm Demonstration

This module provides an educational visualization of pathfinding algorithms
on a 2D grid. Users can create obstacles by clicking, then run Dijkstra's 
algorithm or A* search to see how each algorithm explores the grid and 
finds optimal paths.

Features:
- Interactive obstacle placement
- Real-time algorithm visualization with step-by-step progression
- Performance comparison between Dijkstra and A* algorithms  
- Automatic screenshot generation for analysis
- Adjustable animation speed for detailed observation

Educational Purpose:
Demonstrates the differences in exploration patterns between uninformed 
(Dijkstra) and informed (A*) search algorithms.

@author Martin
@created 2023
@version 1.0.0
"""

import pygame as py
import os, time

class Node():
    """
    Represents a single cell in the pathfinding grid.
    
    Each node stores its position, pathfinding state, and algorithm-specific
    values (g-cost, h-cost) for visualization during search execution.
    """
    
    def __init__(self, x, y):
        """
        Initialize a grid node with coordinates and default state.
        
        Args:
            x (int): Grid column index
            y (int): Grid row index
        """
        self.x = x
        self.y = y
        
        # Pathfinding state flags
        self.focused = False      # Currently being examined by algorithm
        self.explored = False     # Already visited by algorithm
        self.explorable = True    # Can be traversed (not an obstacle)
        
        # Graph connectivity and algorithm values
        self.neighbours = []      # Connected adjacent nodes
        self.color = None        # Visual representation color
        self.g = 0              # Distance from start (Dijkstra/A*)
        self.h = 0              # Heuristic distance to goal (A* only)

    def __str__(self):
        """String representation showing grid coordinates."""
        return f'{self.x,self.y}'

    def add_neighbour(self, node):
        """
        Add connection to adjacent node.
        
        Args:
            node (Node): Neighboring node to connect
        """
        self.neighbours.append(node)
   
    def suppr(self):
        """Remove this node from all neighbor connections."""
        for neighbour in self.neighbours:
            neighbour.neighbours.remove(self)

class Screen():
    """
    Main visualization and interaction system for pathfinding algorithms.
    
    Manages the grid display, user interaction, algorithm execution,
    and visual feedback for educational pathfinding demonstration.
    """
    
    def __init__(self):
        """
        Initialize the interactive pathfinding demonstration.
        
        Sets up display window, grid dimensions, and starts the interface
        ready for obstacle placement and algorithm execution.
        """
        # Display configuration
        self.grid_dimensions = (10, 10)      # 10x10 grid for clear visualization
        self.screen_dimensions = (600, 600)  # Square window for grid display
        self.window = py.display.set_mode(self.screen_dimensions)
        
        # Visual styling
        self.black = (59, 54, 67)  # Dark background color
        self.border_width = 1      # Grid cell border thickness
        
        self.init()

    def init(self):
        """
        Initialize or reset the demonstration to starting state.
        
        Clears the display, resets algorithm results, builds the grid,
        and prepares for new pathfinding demonstration.
        """
        self.window.fill(self.black)
        self.results = []              # Storage for algorithm results
        self.sleep = 10e-2            # Animation delay between steps
        self.build()
        self.draw()
   
    def reset(self):
        """
        Reset grid state while preserving obstacle configuration.
        
        Clears pathfinding data (g/h costs, exploration flags) but keeps
        user-placed obstacles intact for running different algorithms.
        """
        self.window.fill(self.black)
        self.results = []
        
        # Reset pathfinding state while preserving obstacles
        for node in self.nodes:
            node.g = 0
            node.h = 0
            node.explored = False
            node.focused = False
        self.draw()

    def sleep_speed(self, e):
        """
        Adjust animation speed for algorithm visualization.
        
        Args:
            e (int): Speed adjustment factor (+1 faster, -1 slower)
        """
        self.sleep *= 10 ** (-e)
        print(f'Animation Speed: {self.sleep}s per step')

    def build(self):
        """
        Construct the pathfinding grid with nodes and connections.
        
        Creates a 10x10 grid where each node connects to its 8 adjacent
        neighbors (including diagonals). Sets default start and end points.
        """
        self.grid = []   # 2D array for position-based node lookup
        self.nodes = []  # Flat list of all nodes
        
        # Create grid nodes
        for i in range(self.grid_dimensions[0]):
            row = []
            for j in range(self.grid_dimensions[1]):
                node = Node(i, j)
                self.nodes.append(node)
                row.append(node)
            self.grid.append(row)

        # Establish 8-directional connectivity
        for x in range(self.grid_dimensions[0]):
            for y in range(self.grid_dimensions[1]):
                node = self.grid[x][y]
                # Connect to all adjacent cells (including diagonals)
                for i in range(x-1, x+2):
                    for j in range(y-1, y+2):
                        if (0 <= i < self.grid_dimensions[0] and 
                            0 <= j < self.grid_dimensions[1] and 
                            (x,y) != (i,j)):
                            node.add_neighbour(self.grid[i][j])
       
        # Set default start and end positions for clear visualization
        self.start = self.grid[2][2]        # Top-left area
        self.end = self.grid[-2][-2]        # Bottom-right area
       
    def click(self):
        """
        Handle mouse clicks for obstacle placement/removal.
        
        Converts mouse position to grid coordinates and toggles the
        explorable state of the clicked cell (obstacle vs passable).
        """
        x, y = py.mouse.get_pos()
        
        # Convert screen coordinates to grid indices
        width = int(self.screen_dimensions[0] / self.grid_dimensions[0])
        height = int(self.screen_dimensions[1] / self.grid_dimensions[1])
        i, j = x // width, y // height
        
        # Toggle obstacle state and update display
        node = self.grid[i][j]
        node.explorable = not node.explorable
        self.draw_rect(node)

    def draw_rect(self, node):
        """
        Render a single grid cell with appropriate color and text.
        
        Color coding:
        - Orange: Start position
        - Blue: End position  
        - Light blue: Currently focused by algorithm
        - Pale blue: Already explored
        - White: Explorable terrain
        - Dark: Obstacles
        
        Args:
            node (Node): Grid cell to render
        """
        width = int(self.screen_dimensions[0] / self.grid_dimensions[0])
        height = int(self.screen_dimensions[1] / self.grid_dimensions[1])
        x, y = node.x * width, node.y * height
       
        # Determine cell color based on state
        if node == self.start:
            color = (255, 159, 28)      # Orange for start
        elif node == self.end:
            color = (34, 81, 255)       # Blue for end
        elif node.focused:
            color = (133, 159, 255)     # Light blue for current focus
        elif node.explored:
            color = (178, 196, 255)     # Pale blue for explored
        elif node.explorable:
            color = (255, 255, 255)     # White for passable terrain
        else:
            color = (59, 54, 67)        # Dark for obstacles
            
        # Draw cell with border
        py.draw.rect(self.window, color, 
                    (x + self.border_width, y + self.border_width, 
                     width - 2*self.border_width, height - 2*self.border_width))
       
        # Display algorithm values (g-cost and h-cost) for educational purposes
        py.font.init()
        font = py.font.SysFont('démo/Product-Sans-Regular.ttf', 30)
        
        # Show g-cost (distance from start) in top-left
        if node.g > 0:
            text_surface = font.render(str(round(node.g, 1)), False, self.black)
            self.window.blit(text_surface, (x + 2*self.border_width, y + 4*self.border_width))
            
        # Show h-cost (heuristic to goal) in bottom-left for A*
        if node.h > 0:
            text_surface = font.render(str(round(node.h, 1)), False, self.black)
            self.window.blit(text_surface, (x + 2*self.border_width, y + height - 25))
            
        py.display.flip()

    def draw(self):
        """
        Render the complete grid with all cells and their current states.
        
        Called during initialization and reset to display the full grid.
        Also calculates and stores bounding box information for each cell.
        """
        width = int(self.screen_dimensions[0] / self.grid_dimensions[0])
        height = int(self.screen_dimensions[1] / self.grid_dimensions[1])
        
        for i in range(self.grid_dimensions[0]):
            for j in range(self.grid_dimensions[1]):
                x, y = i * width, j * height
                node = self.grid[i][j]
                node.box = [x, y, width, height]  # Store for path drawing
                self.draw_rect(node)
        py.display.flip()

    def draw_path(self):
        """
        Visualize the calculated optimal path as a line overlay.
        
        Draws lines connecting path nodes from start to end, showing
        the final result after algorithm completion.
        """
        for result in self.results:
            # Draw path as connected line segments
            for k in range(len(result['path'])-1):
                pointA = result['path'][k]
                pointB = result['path'][k+1]
                
                # Calculate center points of cells for line endpoints
                xA = pointA.box[0] + pointA.box[2] / 2 
                yA = pointA.box[1] + pointA.box[3] / 2
                xB = pointB.box[0] + pointB.box[2] / 2 
                yB = pointB.box[1] + pointB.box[3] / 2
                
                py.draw.line(self.window, result['color'], (xA, yA), (xB, yB), width=10)
        py.display.flip()

    def distance(self, nodeA, nodeB):
        """
        Calculate Euclidean distance between two nodes.
        
        Used as the cost function for pathfinding algorithms and
        as the heuristic function for A*.
        
        Args:
            nodeA (Node): First node
            nodeB (Node): Second node
            
        Returns:
            float: Euclidean distance between nodes
        """
        return ((nodeA.x - nodeB.x)**2 + (nodeA.y - nodeB.y)**2)** 0.5

    def dijkstra(self):
        """
        Execute Dijkstra's algorithm demonstration with visualization.
        
        Runs the algorithm twice: first to count iterations for screenshot
        timing, then with visual updates and intermediate screenshots.
        """
        self.screenshot()
        
        # First run: count total iterations for screenshot planning
        i = self.dijkstra_algo_iterations(self.start, self.end)
        self.reset()
        
        # Second run: with visualization and screenshots at key points
        n = 3
        iterations = [i * k // n for k in range(n)]  # Screenshot at 0%, 33%, 66%
        distance = self.dijkstra_algo(self.start, self.end, iterations)
        
        # Reconstruct and store the optimal path
        total = distance[self.end]['distance']
        node = self.end
        path = [node]
        while node != self.start:
            node = distance[node]['source']
            path.append(node)
            
        self.results.append({
            'algo': 'Dijkstra',
            'distance': int(total),
            'color': (255, 159, 28),  # Orange path
            'path': path
        })
        
        self.draw_path()
        self.screenshot()

    def dijkstra_algo_iterations(self, start, end):
        """
        Count iterations required for Dijkstra's algorithm completion.
        
        Runs Dijkstra without visualization to determine total iteration
        count for planning screenshot timing in the visual run.
        
        Args:
            start (Node): Starting node
            end (Node): Target node
            
        Returns:
            int: Total number of iterations until completion
        """
        queue = [start]
        distance = {start: {'distance': 0, 'source': None}}
        t = time.time()
        i = 0
        
        while queue:
            i += 1
            current = queue.pop(0)
            current.focused = True
            
            if current == end:
                print(f'Dijkstra execution time: {time.time() - t}s')
                return i
                
            for neighbour in current.neighbours:
                # Only process explorable (non-obstacle) nodes
                if neighbour.explorable:
                    if not neighbour.explored:
                        queue.append(neighbour)
                        neighbour.explored = True
                        
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    
                    # Update if shorter path found
                    if (neighbour not in distance or new_dist < distance[neighbour]['distance']):
                        distance[neighbour] = {'distance': new_dist, 'source': current}
                        neighbour.g = new_dist

    def dijkstra_algo(self, start, end, iterations):
        """
        Dijkstra's algorithm with step-by-step visualization.
        
        Args:
            start (Node): Starting node
            end (Node): Target node
            iterations (list): Iteration numbers for screenshot capture
            
        Returns:
            dict: Distance and source information for path reconstruction
        """
        queue = [start]
        distance = {start: {'distance': 0, 'source': None}}
        i = 0
        
        while queue:
            i += 1
            current = queue.pop(0)
            current.focused = True
            self.draw_rect(current)
            
            # Take progress screenshots at planned intervals
            if i in iterations:
                self.screenshot()
                
            if current == end:
                return distance
                
            for neighbour in current.neighbours:
                if neighbour.explorable:
                    if not neighbour.explored:
                        queue.append(neighbour)
                        neighbour.explored = True
                        
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    
                    if (neighbour not in distance or new_dist < distance[neighbour]['distance']):
                        distance[neighbour] = {'distance': new_dist, 'source': current}
                        neighbour.g = new_dist
                        self.draw_rect(neighbour)  # Update visual display
                        
            time.sleep(self.sleep)  # Animation delay
        
        print(f'Dijkstra completed in {i} iterations')
        return None

    def a_star(self):
        """
        Execute A* algorithm demonstration with visualization.
        
        Similar to Dijkstra demo but uses heuristic guidance for more
        efficient pathfinding with fewer explored nodes.
        """
        self.screenshot()
        
        # Count iterations for screenshot timing
        i = self.a_star_algo_iterations(self.start, self.end)
        print(f'A* will take {i} iterations')
        self.reset()
        
        # Visual run with screenshots
        n = 3
        iterations = [i * k // n for k in range(n)]
        print(f'Screenshots at iterations: {iterations}')
        distance = self.a_star_algo(self.start, self.end, iterations)
        
        # Reconstruct optimal path
        path = []
        node = self.end
        total = distance[self.end]['distance']
        path = [node]
        while node != self.start:
            node = distance[node]['source']
            path.append(node)
            
        self.results.append({
            'algo': 'A*',
            'distance': int(total),
            'color': (255, 159, 28),  # Orange path
            'path': path
        })
        
        self.draw_path()
        self.screenshot()

    def a_star_algo_iterations(self, start, end):
        """
        Count iterations for A* algorithm completion.
        
        Args:
            start (Node): Starting node
            end (Node): Target node
            
        Returns:
            int: Total iterations until goal reached
        """
        queue = [{'node': start, 'priority': 0}]
        distance = {start: {'distance': 0, 'source': None}}
        i = 0
        t = time.time()
        
        while queue:
            i += 1
            current = queue.pop(0)['node']
            
            if current == end:
                print(f'A* execution time: {time.time() - t}s')
                return i
                
            for neighbour in current.neighbours:
                if neighbour.explorable:
                    if not neighbour.explored:
                        neighbour.explored = True
                        
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    
                    if neighbour not in distance or new_dist < distance[neighbour]['distance']:
                        distance[neighbour] = {'distance': new_dist, 'source': current}
                        neighbour.g = new_dist
                        
                        # A* heuristic: straight-line distance to goal
                        h = self.distance(neighbour, self.end)
                        neighbour.h = h
                        priority = new_dist + h  # f(n) = g(n) + h(n)
                        
                        queue.append({'node': neighbour, 'priority': priority})
                        
                    # Keep queue sorted by priority for optimal exploration
                    queue.sort(key=lambda x: x['priority'])

    def a_star_algo(self, start, end, iterations):
        """
        A* algorithm with heuristic guidance and visualization.
        
        Args:
            start (Node): Starting node
            end (Node): Target node
            iterations (list): Iteration numbers for screenshots
            
        Returns:
            dict: Distance and source information for path reconstruction
        """
        queue = [{'node': start, 'priority': 0}]
        distance = {start: {'distance': 0, 'source': None}}
        i = 0
        
        while queue:
            i += 1
            current = queue.pop(0)['node']
            current.focused = True
            self.draw_rect(current)
            
            if i in iterations:
                self.screenshot()
                
            if current == end:
                return distance
                
            for neighbour in current.neighbours:
                if neighbour.explorable:
                    if not neighbour.explored:
                        neighbour.explored = True
                        
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    
                    if neighbour not in distance or new_dist < distance[neighbour]['distance']:
                        distance[neighbour] = {'distance': new_dist, 'source': current}
                        neighbour.g = new_dist
                        
                        h = self.distance(neighbour, self.end)
                        neighbour.h = h
                        priority = new_dist + h
                        
                        self.draw_rect(neighbour)  # Show algorithm values
                        queue.append({'node': neighbour, 'priority': priority})
                        
                    queue.sort(key=lambda x: x['priority'])
                    
            time.sleep(self.sleep)
            
        print(f'A* completed in {i} iterations')
        return None

    def screenshot(self):
        """
        Save current grid state as PNG for documentation/analysis.
        
        Automatically numbers screenshots sequentially in the demo folder.
        """
        path = 'démo/screenshots/'
        filename = f'{path}/image_{len(os.listdir(path))}.png'
        py.image.save(self.window, filename)

# Application entry point and main event loop
screen = Screen()
running = True

print("\nPathfinding Algorithm Demonstration")
print("Controls:")
print("- Click cells to place/remove obstacles")
print("- Press 'D' to run Dijkstra's algorithm")
print("- Press 'A' to run A* algorithm")
print("- Press 'R' to reset (keeps obstacles)")
print("- Press UP/DOWN arrows to adjust animation speed")

while running:
    for event in py.event.get():
        if event.type == py.QUIT:
            running = False
            
        if event.type == py.MOUSEBUTTONDOWN:
            mouse_presses = py.mouse.get_pressed()
            if mouse_presses[0]:  # Left click
                screen.click()
                
        if event.type == py.KEYDOWN:
            if event.key == py.K_r:      # Reset grid
                screen.reset()
            if event.key == py.K_d:      # Run Dijkstra
                screen.dijkstra()
            if event.key == py.K_a:      # Run A*
                screen.a_star()
            if event.key == py.K_UP:     # Speed up animation
                screen.sleep_speed(1)
            if event.key == py.K_DOWN:   # Slow down animation
                screen.sleep_speed(-1)