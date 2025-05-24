# Drone Delivery Optimization Project

## DESCRIPTION
This Python project explores drone delivery route optimization using advanced pathfinding algorithms
and 3D city modeling. It was developed as an individual project to demonstrate how drones can
efficiently navigate urban environments while avoiding buildings and restricted areas.

The project consists of three main components:
1. A pathfinding algorithm demonstration
2. A 3D model-based navigation system
3. A delivery route optimization tool

## FEATURES
- Interactive visualization of pathfinding algorithms (Dijkstra, A*)
- 3D city navigation using real Paris building data
- Delivery route optimization for multiple stops
- Comparison between drone and ground-based delivery routes
- Real-time visualization using Pygame and Open3D
- Building avoidance and altitude-based path constraints

## COMPONENTS
1. Algorithm Demonstration (demo/main_lab.py):
   - Visual demonstration of Dijkstra and A* pathfinding algorithms
   - Step-by-step visualization showing how each algorithm explores nodes
   - Cost calculation and comparison between algorithms
   - Interactive grid for creating obstacles and testing different scenarios

2. 3D City Navigation (drone.py):
   - Loads 3D model of Paris from OBJ files
   - Samples the model to create navigable points
   - Implements Dijkstra and A* algorithms for 3D navigation
   - Visualizes optimal paths in 3D space using Open3D
   - Considers building heights for flight path planning

3. Delivery Optimization (drone_delivery.py):
   - Allows user to select multiple delivery locations on a map
   - Calculates optimal route to visit all locations
   - Compares drone delivery with ground-based delivery
   - Visualizes paths in both 2D and 3D
   - Considers permutations of delivery order for true optimization

## USAGE
1. Algorithm Demonstration:
   > python demo/main_lab.py
   
   Controls:
   - Mouse Click: Create/remove obstacles
   - D: Run Dijkstra algorithm
   - A: Run A* algorithm
   - R: Reset the grid
   - UP/DOWN: Adjust simulation speed

2. 3D Navigation Demo:
   > python drone.py
   
   This will load the Paris 3D model and calculate a path between two random points.

3. Delivery Optimization:
   > python drone_delivery.py
   
   Steps:
   - Enter the number of delivery points when prompted
   - Click on the map to select the depot (first click)
   - Click to select delivery locations
   - The program will calculate and display the optimal route
   - S: Take screenshot
   - R: Reset and start over

## TECHNICAL DETAILS
The project uses several key technologies and techniques:

- Data Structures:
  * Graph representation of navigable space
  * Priority queues for algorithm optimization
  * Permutation generation for route optimization

- Algorithms:
  * Dijkstra's algorithm for shortest path finding
  * A* algorithm with heuristic distance estimation
  * Traveling Salesman Problem (TSP) optimization for delivery route

- 3D Processing:
  * Open3D for 3D model loading and visualization
  * Point cloud sampling of building meshes
  * Altitude-based navigation constraints

- Visualization:
  * Pygame for 2D map visualization and interaction
  * Open3D for 3D path visualization
  * Color-coded paths and exploration visualization

## DATA SOURCES
- 3D building models of Paris
- Terrain elevation data
- Aerial imagery for 2D map overlay

## IMPLEMENTATION
The code is organized into three main classes:

1. Node: Represents points in space with:
   - 3D coordinates
   - Grid indices
   - Neighbor connections
   - Pathfinding metadata

2. City/Screen: Manages the environment with:
   - Data loading and preprocessing
   - Graph building and connection
   - Algorithm implementation
   - Visualization control

3. Drone: Represents the delivery vehicle with:
   - Flight altitude constraints
   - Navigation capabilities

## RESULTS
The project demonstrates that:
- A* algorithm consistently outperforms Dijkstra for drone path planning
- Drones can achieve significantly shorter delivery routes compared to ground vehicles
- 3D navigation constraints have a major impact on optimal route selection
- Multi-stop delivery requires sophisticated ordering optimization

## FUTURE IMPROVEMENTS
- Weather condition integration
- Dynamic obstacle avoidance
- Battery life and charging station considerations
- Integration with actual drone control systems
- More sophisticated TSP algorithms for larger delivery sets
- Real-time traffic data for ground vehicle comparison

## DEPENDENCIES
- Python 3.x
- NumPy
- Pygame
- Open3D
- PyOpenGL (for some 3D visualization)
