.. slope_constrained_planner documentation master file, created by
   sphinx-quickstart on Mon May 15 19:15:01 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Slope-Informed Global Path Planning for Quadruped Robots.
==============================================================================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:


Overview
==========
This work describes the initial steps in developing a global path planner for a quadruped robot designed
to traverse outdoor environments with uneven terrains. The ultimate goal is to generate paths that strike
a balance between path length, tortuousness, energy efficiency, and safety when encountering slopes. 
The article details the system architecture and the implemented planning method, which incorporates 
slope constraints into a roadmap-based approach to generate paths with various characteristics. The 
algorithm has undergone extensive testing, both in simulation and with the Spot robot from Boston 
Dynamics. In both sets of experiments, noticeable differences were observed when adjusting the constraints 
on the robot’s maximum allowable inclination angles.

Authors:

* Ali Yousefi, ali.yousefi@edu.unige.it
* Zoe Betta, zoe.betta@edu.unige.it
* Giovanni Mottola, giovanni.mottola@unige.it
* Antonio Sgorbissa, antonio.sgorbissa@unige.it
* Carmine Tommaso Recchiuto, carmine.recchiuto@unige.it

©2024 RICE - DIBRIS, University of Genova

.. image:: diagrams/rice_logo.jpg
  :width: 150
  :align: left
  :alt: rice_logo


|
|
|
|
|
|


Contents
==========
.. toctree::
   :maxdepth: 1
   
   simulation_environment
   real_world
   software_architecture
   sequence_diagram
   source_files
   usage
   working_hypothesis
   
   
   
Indices and tables
==================
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
