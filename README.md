# Better than Apple Maps
#### ECE 470 - Spring 2020

Navid Mokhlesi (navidm2)

Tori Colthurst (vrc2)

Jing-Chen Peng (jcpeng)

## Summary
This project is to control a simulated robot to move around in a 3D world, 
avoiding obstacles and building a 2D represntation of its world while it moves. 
It can search through its world to locate and pick up items, and deliver them 
to a predefined location. The motivation behind this is a ‘recycling robot’ 
that picks up trash and throws it away.

## Getting Started

Clone the repository.

Install these guys:

* [CoppeliaSim](https://www.coppeliarobotics.com/) - The simulation thing
* [Python](https://python.org) - Python 3

Python requirements:
* numpy
* scipy
* matplotlib (for now, for visualizing/debugging the mapper)

Install these by running:

```
pip install numpy scipy matplotlib
```

or

```
pip3 install numpy scipy matplotlib
```

Start CoppeliaSim. Open the scene (robot_lidar_test.ttt)

Run the simulation:

```
python3 main_sim.py
```

or

```
python3 main_sim.py
```

## Current script files:

* robot_motion.py: Robot 2d base movement demonstrated here, can follow waypoints
* arm_motion.py: Robot arm movement and foward kinematics to determine end effector state.
* robot_lidar.py: Reading & interpreting sensor data for analysis.
* robot_localization.py: Start on particle filter based localization leveraging lidar data.

## TODO / next steps:
* Interpret lidar data correctly
* Make the arm move and pick up blocks
* Map while moving :O

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.
