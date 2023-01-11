# path-planning-airsim-scripts
- Python scripts for path planning and obstacle detection and avoidance using LiDAR sensor in an AirSim - Unreal Engine simulation.
  
- A* is used to identify the optimal path

- To install, simply drop the files in your AirSim/Pythonclient/multirotor folder or in your environment folder.

- Also, the "drone_program_3d.py" has only one episode, which is determined by the user. At the end the drone returns back to its original positions following the reversed optimal path it found.

- The "drone_astar_3d_re.py" script needs a .csv file and the astar_algo.py to run. The .csv contains 1000 random start/end points for the drone to explore. In addition the area_definer.py defines a limit to the map so the drone does not overshoot. After each episode a .csv file named metrics is created to store its results.

- It is important to note that the 3-D A* algorithm that is used comes from https://github.com/fvilmos/simpleAstar of the user fvilmos.
