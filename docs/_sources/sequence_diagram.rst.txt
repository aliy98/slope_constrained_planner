Sequence Diagram
==================

The sequnce diagram of this experiment is represented in the following figures.

Simulation:

.. image:: diagrams/sequence.png
  :width: 1000
  :align: center
  :alt: sequence
  

Real world:

.. image:: diagrams/sequence2.png
  :width: 1000
  :align: center
  :alt: sequence2
  

In the first loop, in which the path planning occurs, the planner gets the goal
state as an action goal, elevation map data, and robot’s current state. Once the
solution is found, it triggers the second loop, which is for path following task.
In simulation mode, ``path_follower`` would act as local planner, using a PID controller, 
while in the real world experiment, ``goal_publisher`` sends the next waypoint to the Spot robot's 
available local planner. 