# EECS206A
Introduction to Robotics Final Project

### Introduction
BaleBot aims to coordinate multiple TurtleBots to act as an arbitrary rigid body. It will plan and track a trajectory from an initial state to a final state while maintaining its rigid body structure. This is an interesting project because it deals with planning and control of a nonlinear, nonholonomic system. To make BaleBot work, it has to generate a path that takes into account system dynamics while synchronizing multiple control loops. This project would be useful for warehouse robotics. It can allow for transportation of large objects with multiple smaller robots instead of a single large robot.

### Design
The primary design criteria for our project to meet is to maintain the rigid body structure. This is determined by the distance and orientation of each auxiliary robot with respect to a primary robot. We settled on a hierarchical control scheme where a global controller generates controller inputs for the group while a local control translates the global control inputs to each individual robot. Treating formation as a controls problem allowed us to design for robustness. However this limited our robot configuration to be along the horizontal axis of the primary robot. For path planning, we utilized an algorithm from the Sapienza University of Rome that was designed specifically for unicycle-model robots. This algorithm allowed us to generate smooth paths very efficiently. The generated trajectory was then sampled and sent to the controller node. Selecting too few waypoints would reduce the resolution of the path while sampling too many waypoints would result in missed waypoints due to the system not responding fast enough.

### Implementation

In order to make the system as robust as possible, we defined on turtlebot as the "master" turtlebot, and used it as the center of our multirobot ridid body. This master turtlebot is in charge of following the path sent by the pathplanner function. 
The slave robots do not "now" what the path is, their controller only tries to make them keep their position & orientation relative to the master.
All bots use the state observer as feedback, constantly moniroting their positions relative to where they should be.

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

We obtained a robust, modular and versatile system that is capable of having an arbitrary number of robots follow almost any path while behaving as a rigid body.

	* The slave controllers behaved incredibly well and were extremely robust to disturbances in both the initial conditions (i.e the turtlebots not being initially positionned in the required confirguration) and unforseen errors in the master's behavior. The slave robots would adhere to their theoretical coordinates within 5cm and 5 degrees of error, which is the best possible reacheable precision with the available hardware.

	* The master controller will behave robustly on the majority of ordinary paths (for instance 0 to 180 curves, S-shaped paths). It will reach its goal with less than 10cm or error, and always positionned within 5 degrees from the desired orientation.

### Possible improvements

Concerning the slaves: we would like to make them more reactive to abrupt decisions of the master. For example, if the master suddently stops for a mechanical reason, the slaves will move another 5cm before stopping, an maintain that steady state error as long as the master has not restarted. To correct that, we would want to implement a PID controller instead of the current PD controller and tune the gains appropriatly.

The master is in charge of following the path and making sure that the whole rigid body stays on course. This can be improved in a few ways. First, we would like to adapt the controller to make it more robust and be able to follow all paths, irrespective of shape. As of now, the controller takes the distance between the master and the next waypoint as input. This can introduce instability when the master narrowly passes and misses a waypoint. The controller will make it accelerate and miss its next waypoint. To solve that, we want to use the signed distance to the waypoint. This would require significanlty more calculation and tf transformation, which would make the system less responsive. It would greatly improve robustness for difficult paths, however.

Other potential improvements include: using onboard sensors instead of an external camera for a more mobile solution, incorporating odometry data for a better state estimate, allowing for arbitrary configurations of the rigid body, and adding obstacle avoidance capabilities.

### Conclusion
### Team
#### Ayusman Saha (EECS M.Eng.)
Worked on simulation, state observer, path planner, and control
#### Léo Toulet
Worked on frame publisher, path planning and control.
#### Philippe De Sousa
#### Tarun Singh
Worked on perception
#### Joshua Lin
Did not do work

### Additional Materials
[Source Code](https://github.com/TheYoshiStory/EECS206A)

[Demo Video](https://www.youtube.com/watch?v=kbM0s6gdCPo&feature=youtu.be&fbclid=IwAR2qxp5GAcLNqVgVhX4M-gtqX_T4E1JtNcoWQ8MlfFs21jSxS7CIz3DUggI)
