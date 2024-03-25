
# A* Pathfinding in Obstacle Space

This Python script implements the A* search algorithm to find an optimal path for a robot navigating through a specified obstacle space. The obstacle space is defined with specific obstacles, including rectangles and a hexagon, with added functionalities such as obstacle clearance and robot radius consideration. The script generates a visualization of the explored nodes, the final path taken, and a video showing the pathfinding process.


# Authors
Nazrin Gurbanova, UID: 120426469, Directory ID: nazrin

Onkar Kher, UID: 120407062, Directory ID: okher  

## Features

- **Obstacle Space Definition**: Includes predefined obstacles in a 2D space where the robot cannot traverse.
- **Dynamic Obstacle Clearance and Robot Radius**: Users can input the obstacle clearance and the robot's radius to adjust the navigable space dynamically.
- **A\* Pathfinding Algorithm**: Utilizes the A* algorithm to find the shortest path from a start point to a goal point, considering the robot's orientation.
- **Visualization and Video Generation**: Outputs visualizations of the obstacle space, the explored nodes, the optimal path, and a video showing the step-by-step pathfinding process.

## Dependencies

- Python 3.x
- NumPy
- Matplotlib
- OpenCV-Python
- Heapq (Python standard library)

Ensure these dependencies are installed before running the script. They can typically be installed via pip:

```sh
pip install numpy matplotlib opencv-python
```

## Usage

1. **Setting Up**: Clone the repository or download the script to your local machine. Ensure all dependencies are installed.

2. **Running the Script**: Navigate to the script's directory in your terminal and run:

    ```sh
    python a_star_nazrin_onkar.py
    ```

3. **Input Parameters**: The script will prompt you for several inputs:
   - Obstacle Clearance
   - Robot Radius
   - Robot Step Size
   - Start Node Coordinates (x, y)
   - Start Node Orientation
   - Goal Node Coordinates (x, y)
   - Goal Node Orientation

4. **Visualization and Output**: Upon successful execution, the script generates:
   - A visualization plot saved as an image for each step, showing the explored nodes and the path.
   - A video `output_video.mp4` demonstrating the pathfinding process.

## Implementation Details

- **Node Representation**: The script uses a `Node` class to represent each state with attributes for coordinates, orientation, cost, and parent node.
- **Action Set**: Defines possible moves for the robot with specified orientations and step sizes, calculating the new state and cost.
- **Obstacle Space**: Initializes a 2D array representing the obstacle space, marking obstacles and boundaries based on the given clearance and robot radius.
- **A\* Algorithm**: Implements the A* search algorithm to explore nodes, using a priority queue to select the next node based on the lowest cost + heuristic value.
- **Backtracking and Visualization**: Once the goal node is reached, the script backtracks from the goal to the start to determine the path taken and generates visualizations.

## Limitations and Recommendations

- The obstacle space and obstacles are predefined; customizing them requires manual adjustments to the script.
- The visualization and video generation are optimized for the given obstacle space dimensions. Adjusting the space size may require modifications to the plotting functions for optimal visualization.
- The script assumes a 2D planar space with specific robot motion constraints. Expanding it to more complex scenarios or 3D spaces would require significant modifications.

## License

This project is open-sourced under the MIT License. Feel free to use, modify, and distribute the code as per the license conditions.


---
