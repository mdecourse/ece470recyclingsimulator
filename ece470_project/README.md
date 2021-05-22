# Better than Apple Maps
#### ECE 470 - Spring 2020

Navid Mokhlesi (navidm2)

Tori Colthurst (vrc2)

Jing-Chen Peng (jcpeng)

## Summary
The goal of this project is to simulate a robot in a 3D world using Coppeliasim. 
The robot will build a 2D representation of its world while it moves around the open
space, avoiding obstacles. It will move about this environment in search of 'trash' and
'recycling' items on thr ground. The robot will be able to detect, move to, pick up,
and deliver these items to a predefined location. The motivation for this robot is 
a 'recycling robot' that can be put in any space to pick up litter and dispose of it.

## Getting Started

Clone the repository.

Install these guys:

* [CoppeliaSim](https://www.coppeliarobotics.com/) - The simulation thing
* [Python](https://python.org) - Python 3

Python requirements:
* numpy
* scipy
* matplotlib (for visualizing/debugging the mapper)

Install these by running:

```
pip install numpy scipy matplotlib
```

or

```
pip3 install numpy scipy matplotlib
```

Start CoppeliaSim. Open the environment scene (maze_test.ttt)

Run the simulation:

```
python main_sim.py
```

or

```
python3 main_sim.py
```

## Current script files:

* robot_motion.py: Class definitions and functions for 2d base motion of the robot. 
* arm_motion.py: Class definitions and functions for robot arm motion, including forward and inverse kinematics.
* robot_lidar.py: Class definitions and functions for reading and interpreting LIDAR sensor data.
* robot_localization.py: Class definitions and functions for the particle filter, and input space map.
* vision.py: Class definitions and functions for reading and interpreting vision sensor data.
* pf_test.py: Helper functions for testing the particle filter outside of the CoppeliaSim environment.
* utils.py: Helper functions for 2D and 3D matrix rotation/translation manipulations, and some geometry stuff.
* dijkstra_hardcode.py: Precomputes paths between every pair of points in the (discretized) workspace. Resulting map is used for pathfinding.
* robot_lidar_test.ttt: Old environment file.

## TODO / next steps:
* Move to SLAM instead of human-input map.
* Improve the path planning (right now it just assumes the robot is a circle and tends to crash).

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.
