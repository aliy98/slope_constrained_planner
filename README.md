## Slope-Informed Global Path Planning for Quadruped Robots

This work describes the initial steps in developing a global path planner for a quadruped robot designed to traverse outdoor environments with uneven terrains. The ultimate goal is to generate paths that strike a balance between path length, tortuousness, energy efficiency, and safety when encountering slopes. The article details the system architecture and the implemented planning method, which incorporates slope
constraints into a roadmap-based approach to generate paths with various characteristics. The algorithm has undergone
extensive testing, both in simulation and with the Spot robot from Boston Dynamics. In both sets of experiments, noticeable
differences were observed when adjusting the constraints on the robot’s maximum allowable inclination angles.
Please refer to the provided [documentation](https://aliy98.github.io/slope_constrained_planner/), for more details about this work.

**Authors:** 
  - Ali Yousefi, ali.yousefi@edu.unige.it
  - Zoe Betta, zoe.betta@edu.unige.it
  - Carmine Tommaso Recchiuto, carmine.recchiuto@dibris.unige.it
  - Antonio Sgorbissa, antonio.sgorbissa@unige.it
    
©2024 RICE - DIBRIS, University of Genova
<p align="left">
<img src="https://github.com/aliy98/slope_constrained_planner/assets/65722399/720cd6f1-419a-46f1-82e4-84ae9e84bdd2" width="150" title="rice_logo">
</p>


### Docker Image
There is a docker image provided for this work, containing ROS noetic on Ubuntu 20.04, and all the required packages as well. It could be accessed using the link [HERE](https://hub.docker.com/r/aliy98/slope_constrained_planner). Otherwise, in order to run this package in a native linux, the following dependencies have to be taken into account.

### Dependencies
* The software for this planner is based on [OMPL](https://ompl.kavrakilab.org/index.html), which consists of many state-of-the-art sampling-based motion planning algorithms. The required dependencies could be installed using the following command:
 
```
sudo apt install ros-noetic-ompl ros-noetic-grid-map-core ros-noetic-actionlib ros-noetic-geometry-msgs ros-noetic-grid-map-msgs ros-noetic-grid-map-ros ros-noetic-nav-msgs ros-noetic-roscpp ros-noetic-tf2-geometry-msgs ros-noetic-tf2-ros
```

* Additionally, [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) ROS package was used to create a 2.5D map of the environment. 

* In order to test the software package in simulation environment, [WoLF](https://github.com/graiola/wolf-setup) was used. This package provides whole-body controller, along with the robot description files and some interesting worlds for Boston Dynamics Spot robot, as well as several other famous quadruped robots.


### Usage in Simulation
In order to run the simulation environemnt, along with the elevation mapping package, the following launch file could be used:

```
   roslaunch slope_constrained_planner_ros simulation.launch
```

Once the robot is spawned in a random point by ``go0.py`` script, a random goal point would be chosen on the map with a particular distance to robot. Then, robot would align to the goal point, and it would tilt along it's y-axiz by ``tilt_robot.py`` script, in order to have a better view in the elevation map.

The planner node, could be launched using the following command:

```
   roslaunch slope_constrained_planner_ros planner.launch 
```

Once the solution path is found by the planner, the robot could move along the path, using the provided ``path_follower.py`` script:

```
   roslaunch slope_constrained_planner_ros path_follower.launch
```

### Usage in real-world experiment
Regarding the real-world usage with Boston Dynamics Spot CORE, the elevation mapping package, and spot ros packages could be launched using the command:

```
   roslaunch slope_constrained_planner_ros navigation.launch
```

Moreover, the following command would make the robot stand up and wait for the waypoint on the found trajectory. Actually, it uses the commands on the Spot SDK, to perform the task of local path planning, based on the found global path by our planner.

```
   rosrun slope_constrained_planner_ros gotopoint.py
```
The planner node, could be launched using the following command:

```
   roslaunch slope_constrained_planner_ros planner.launch 
```
Once the path is found, the waypoints on the trajectory would be published to the local path planner ``gotopoint``, using the following command:
```
   rosrun slope_constrained_planner_ros goal_publisher.py
```

### Configuration
The parameters of planner components (e.g. sampler, motion validartor, planning algorithm), could be modified in the file ``slope_constrained_planner/config/params.yaml``. Regarding the elevation map, the configuration files are located in the same directory which are named ``map_sim.yaml`` and ``map_real.yaml``.

### System hypothesis and future work
We implemented a global path planner algorithm able to
take into account constraints on the slope a quadruped robot
can face outdoor. The presence slope constraints effectively
influences the generated path in a way that allows for longer
paths but limited slope or vice versa. We observed these
results both in simulation and in real-world experiments with
the Spot robot from Boston Dynamics.

In future work, we plan to conduct more extensive testing
with the Spot robot on steeper hills to thoroughly assess
the algorithm’s limitations. Additionally, we will investigate
the impact of various solutions on battery consumption to
determine potential differences in energy efficiency. This
research will ultimately contribute to the development of a
system that emulates human versatility in making decisions
regarding slope navigation. The robot may opt to follow a
longer (and potentially safer) path or a shorter (but more
energy-intensive) path, taking into account factors such as
task requirements, time constraints, payload, battery charge,
and other relevant parameters.





