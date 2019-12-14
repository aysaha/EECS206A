# EECS206A
Introduction to Robotics Final Project

### Introduction
BaleBot aims to coordinate multiple TurtleBots to act as an arbitrary rigid body. It will plan and track a trajectory from an initial state to a final state while maintaining its rigid body structure. This is an interesting project because it deals with planning and control of a nonlinear, nonholonomic system. To make BaleBot work, it has to generate a path that takes into accounts system dynamics while synchrnoziing multiple control loops. This project would be useful for warehouse robotics. It can allow for transportation of large objects with multiple smaller robots instead of a single large robot.

### Design
The primary design criteria for our project to meet is to maintain the rigid body structure. This is determined by the distance and orientation of each auxiliary robot with respect to a primary robot. We settled on a hierarchical control scheme where a global controller generates controller inputs for the group while a local control translates the global control inputs to each individual robot. Treating formation as a controls problem allowed us to design for robustness. However this limited our robot configuration to be along the horizontal axis of the primary robot. For path planning, we utilized an algorithm from the Sapienza University of Rome that was designed specifically for unicycle-model robots. This algorithm allowed us to generate smooth paths very efficiently. The generated trajectory was then sampled and sent to the controller node. Selecting too few waypoints would reduce the resolution of the path while sampling too many waypoints would result in missed waypoints due to the system not responding fast enough.

### Implementation
`src/frame_publisher.py`
refreshes key coordinate frames in case they get obstructed from view

`src/state_observer.py`
provides state estimations of all coordinate frames

`src/path_planner.py`
plans a smooth trajectory from an initial state to a final state

`src/motion_controller.py`
determines control inputs to maintain rigid body and track trajectory

`msg/State.msg`
fully describes the state-space with (x, y, θ)

`launch/robot.launch`
launches frame_publisher.py, state_observer.py, path_planner.py, motion_controller.py along with 2 static transform publisher nodes. Also contains parameters such as turtlebot color, AR tag numbers, and robot configuration

`launch/simulator.launch`
launches state_observer.py, path_planner.py, motion_controller.py in turtlesim

`launch/rviz.launch`
launches rviz, ar_track_alvar, and usb_cam

### Results
### Conclusion
### Team
#### Ayusman Saha (EECS M.Eng.)
Worked on simulation, state observer, path planner, and control
#### Léo Toulet
#### Philippe De Sousa
#### Tarun Singh
#### Joshua Lin

### Additional Materials
[Source Code](https://github.com/TheYoshiStory/EECS206A)

[Demo Video](https://www.youtube.com/watch?v=_QTUjNOHIRY)
