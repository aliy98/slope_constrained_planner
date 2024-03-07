Temporal Diagram (UML Sequence Diagram)
-----------------------------------------
The following figure represents the UML sequence diagram of this package. In the initial state, user launches the nodes,
``finite_state_machine`` calls ``move_arm`` service to initialze robot arm movement using ``moveit``. Markers 
get scanned by robot camera and for each marker ``image_id`` would be detected. ``finite_state_machine`` requests ``room_info``
by sending room id and build semantic map in the ontology. Then ``base_movement_state`` gets true by ``finite_state_machine`` node
for enabling ``robot-states`` node to simulate battery consumption and target room position would be sent to ``moveit``
node to find the path and move the robot base. Once the robot gets to target position ``base_movement_state`` gets false
and robot starts exploring the room like the beginning of the sequence. 

.. image:: diagrams/sequence.png
  :width: 800
  :align: center
  :alt: robot_urdf