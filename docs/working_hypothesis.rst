System hypothesis and future work
======================================


We implemented a global path planner algorithm able to take into account constraints on the slope a 
quadruped robot can face outdoor. The presence slope constraints effectively influences the generated 
path in a way that allows for longer paths but limited slope or vice versa. We observed these results 
both in simulation and in real-world experiments with the Spot robot from Boston Dynamics.

In future work, we plan to conduct more extensive testing with the Spot robot on steeper hills to 
thoroughly assess the algorithm’s limitations. Additionally, we will investigate the impact of various 
solutions on battery consumption to determine potential differences in energy efficiency. This research 
will ultimately contribute to the development of a system that emulates human versatility in making 
decisions regarding slope navigation. The robot may opt to follow a longer (and potentially safer) path 
or a shorter (but more energy-intensive) path, taking into account factors such as task requirements, 
time constraints, payload, battery charge, and other relevant parameters.
