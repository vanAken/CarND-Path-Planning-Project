# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Status
This is the first version with a planning STL-A* above the rubrics.  
400m horizont
120s in the future  
4x4m discrete areas 
3 lanes 
each secound an update from the planner

### Model Documentation

##main.cpp
# either recalculate a* or calculate the trajectory
In line 81 there is a decision about the time planning. Behind here is a very important experience about the latency of A* planning.
After a long calculation time of about 0.1 sec. the position data is not correct anymore and the calculated trajectory results in jerks. The solution of this problem is already anchored in the problem definition, but nevertheless everyone has to make this experience to understand it: Trajectory calculation or rule interventions are strictly forbidden after a long algorithm because the data is no longer up-to-date.

##prediction.h
When the prediction object gets initialized, the global time_road will be created. This is a stack of discrete maps for the A* algorithm with the dimensions s, d and t, filled with ones, the cost of a movement on the maps and through the stack. An 8-9-8 means that there is another vehicle with a length of 3(12m) and a width of 1(4m) and this field must not be used. In front and behind there is a potential field with the values
 3,3,2,1 and one right next to it, which in case of doubt shall regulate overtaking. 
It is most important that the old path of the previous planning is filled with zeros so that it is preferably reused.

The search method initializes and starts A*. If no result was found, A* is started with a shorter horizon. If this is not successful, a follow mode calculates a path without strip change with safe distance to the front vehicle, so that in any case the vectors for s d and v are stored globally at intervals of one second for the corresponding horizon.


##MapSearchNode.h
MapSearchNode contains all external functions of the 
A* Algorithuses from the standard C++ Biothek(STL) by Justin Heyes-Jones. The library works according to the principle of the priority queue in which all generated nodes have a status vector and are sorted according to costs.

In the function "MapSearchNode::GetSuccessors" the decided rule is defined, when a new nodes is created and which status it should have.

For example, five fields can be moved forward in one second with v_max, if they are free - the costs must be added up for the status of the new node.
Strip changes cost twice as much once, on both sides of the road marking.


The costs are defined in "MapSearchNode::GetCost":
The movement costs from the time_road are already available as status variable, so that only the time factor was selected as the difference between v_max and v. 
c + 2*(v_max-v) 
This weighting allows the ego car to drop back when it reaches its destination faster.

Source: https://github.com/justinhj/astar-algorithm-cpp

##trajectory.h

Based on the first two points of the old trajectory, the length, the velocity are determined (line 28,29). and the direction for the new trajectory is adopted in the form of the two points and the velocity for the new spline construction is taken over (line32-34).

The next four points are determined by averaging 3 A* points in d direction and speed-dependent points in S direction (line 44-81). These and all other A* points are projected back in Cartesian coordinates and moved into the corresponding output vector (line 88-99) as double points with velocity specification and relative S-value in order to generate the splines (line 105-110). Finally the waypoints are generated on these (line 112-136), but The maximum acceleration in axial and lateral direction must be maintained.

For safety reasons it is checked before a lane change whether this is still free, because A* is not able to output drivable trajectories, because of the latency time, as well as the architecture. For this purpose the method "Trajectory::calc_gap" (line 139) is used.

##helpers.h
Here are the helper, like global variabls and functions for A* including the converting functions

##frenet.h
This object is a very reusability object. It reads in all waypoints and describes the optionally closed track with splines. The methods can convert cartesian coordinates into Frenet coordinates and vice-versa. 
Source: The algorithms come from a slack post by Eric Lavigne "#t3-p-path-planning" from 13 May 2018.

##spline.h
simple cubic spline interpolation library without external dependencies
Source: https://kluge.in-chemnitz.de/opensource/spline/




