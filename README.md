# EECS206A
Introduction to Robotics Final Project

### Introduction
BaleBot aims to coordinate multiple TurtleBots to act as an arbitrary rigid body. It will plan and track a trajectory from an initial state to a final state while maintaining its rigid body structure. This is an interesting project because it deals with planning and control of a nonlinear, nonholonomic system. To make BaleBot work, it has to generate a path that takes into accounts system dynamics while synchrnoziing multiple control loops. This project would be useful for warehouse robotics. It can allow for transportation of large objects with multiple smaller robots instead of a single large robot.

### Design
The primary design criteria for our project to meet is to maintain the rigid body structure. This is determined by the distance and orientation of each auxiliary robot with respect to a primary robot. We settled on a hierarchical control scheme where a global controller generates controller inputs for the group while a local control translates the global control inputs to each individual robot. Treating formation as a controls problem allowed us to design for robustness. However this limited our robot configuration to be along the horizontal axis of the primary robot. For path planning, we utilized an algorithm from the Sapienza University of Rome that was designed specifically for unicycle-model robots. This algorithm allowed us to generate smooth paths very efficiently. The generated trajectory was then sampled and sent to the controller node. Selecting too few waypoints would reduce the resolution of the path while sampling too many waypoints would result in missed waypoints due to the system not responding fast enough.

### Implementation
### Results
### Conclusion
### Team
#### Ayusman Saha (EECS M.Eng.)
Worked on simulation, state observer, path planner, and control
#### Léo Toulet
#### Philippe De Sousa
#### Tarun Singh
#### Joshua Lin

### Additioanl Materials
