Usage
=====

Dependencies
--------------
* The software for this planner is based on `OMPL <https://ompl.kavrakilab.org/index.html>`_, which consists of many state-of-the-art sampling-based motion planning algorithms. The required dependencies could be installed using the following command:
 
.. code-block:: console

   sudo apt install ros-noetic-ompl ros-noetic-grid-map-core ros-noetic-actionlib ros-noetic-geometry-msgs ros-noetic-grid-map-msgs ros-noetic-grid-map-ros ros-noetic-nav-msgs ros-noetic-roscpp ros-noetic-tf2-geometry-msgs ros-noetic-tf2-ros

* Additionally, `elevation_mapping <https://github.com/ANYbotics/elevation_mapping>`_  ROS package was used to create a 2.5D map of the environment. 

* In order to test the software package in simulation environment, `WoLF <https://github.com/graiola/wolf-setup>`_ was used. This package provides whole-body controller, along with the robot description files and some interesting worlds for Boston Dynamics Spot robot, as well as several other famous quadruped robots.

Once the dependencies are met, the package can be installed as it follows:

.. code-block:: console

   mkdir -p catkin_ws/src
   cd catkin_ws/src
   git clone https://github.com/aliy98/slope_constrained_planner
   cd ..
   source /opt/ros/<distro>/setup.bash
   catkin_make

Usage in Simulation
----------------------

In order to run the simulation environemnt, along with the elevation mapping package, the following 
launch file could be used:

.. code-block:: console

   roslaunch slope_constrained_planner_ros simulation.launch

Once the robot is spawned in a random point by ``go0.py`` script, a random goal point would be chosen 
on the map with a particular distance to robot. Then, robot would align to the goal point, and it would 
tilt along it's y-axiz by ``tilt_robot.py`` script, in order to have a better view in the elevation map.

The planner node, could be launched using the following command:

.. code-block:: console

   roslaunch slope_constrained_planner_ros planner.launch 

Once the solution path is found by the planner, the robot could move along the path, using the provided 
``path_follower.py`` script:

.. code-block:: console

   roslaunch slope_constrained_planner_ros path_follower.launch


Usage in Real-World
----------------------

Regarding the real-world usage with Boston Dynamics Spot CORE, the elevation mapping package, and spot 
ros packages could be launched using the command:

.. code-block:: console

   roslaunch slope_constrained_planner_ros navigation.launch


Moreover, the following command would make the robot stand up and wait for the waypoint on the found 
trajectory. Actually, it uses the commands on the Spot SDK, to perform the task of local path planning, 
based on the found global path by our planner.

.. code-block:: console

   rosrun slope_constrained_planner_ros gotopoint.py

The planner node, could be launched using the following command:

.. code-block:: console

   roslaunch slope_constrained_planner_ros planner.launch 

Once the path is found, the waypoints on the trajectory would be published to the local path planner 
``gotopoint``, using the following command:

.. code-block:: console

   rosrun slope_constrained_planner_ros goal_publisher.py

Configuratrion
----------------

The parameters of planner components (e.g. sampler, motion validartor, planning algorithm), could be 
modified in the file ``slope_constrained_planner/config/params.yaml``. Regarding the elevation map, the 
configuration files are located in the same directory which are named ``map_sim.yaml`` and ``map_real.yaml``.