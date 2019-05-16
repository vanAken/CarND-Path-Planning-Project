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
This is the final version of the path planning project with a scheduling AWL Astar algorithm.

300 m horizon

60s in the future

4x4m discrete areas

1s discretization and every 0.2 seconds an update from the planner

3 lanes

# Model documentation
## main.cpp
### either recalculate A* or calculate the trajectory
In line 81 the decision is made on the timing. Behind this is a very important experience about the latency of Astar planning.
After a long computing time of approx. 0.1 sec. The position data are no longer correct and the calculated path leads to jerking. The solution to this problem is already anchored in the problem definition, but still everyone must have this experience to understand it: Path calculations or rule interventions are useless if the data is outdated.

## prediction.h
When the prediction object is initialized, the global timeline is created. This is a stack of discrete maps for the Astar algorithm with dimensions s, d, and t filled with ones, the cost of movement on the maps, and through the stack. A 8-9-8 means that there is another vehicle with a length of 3 (12 m) and a width of 1 (4 m) and this field may not be used. Front and back there is a potential field with the values ​​3,3,2,1 and a right next to it, which governs overtaking in case of doubt. It is important that the old path of the previous planning is filled with zeros, so that it is preferably used.

The search method is initialized and starts Astar. If no result is found, Astar starts with a shorter horizon. If this is unsuccessful, a tracking mode calculates a safe distance path to the forward vehicle so that in any case the vectors for s d and v are stored globally at one second intervals for the corresponding horizon.


## MapSearchNode.h
MapSearchNode contains all external functions of the
Astar algorithms from the Standard C ++ STL by Justin Heyes-Jones. The library operates on the principle of the priority queue, where all generated nodes have a status vector and are sorted by cost.

The MapSearchNode :: GetSuccessors function determines when a new node is created and what status it should have.

For example, with v_max, five fields can be moved forward in one second if they are free - the cost must be added to the status of the new node.
Strip changes cost double on both sides of the lane marking.


The cost is defined in MapSearchNode :: GetCost:
The movement costs from the time_road are already available as status variables, so that only the time factor was selected as the difference between v_max and v.
c + 2 * (v_max -v)
This weighting can cause the ego car to fall back when it reaches its destination faster.

Source: https://github.com/justinhj/astar-algorithm-cpp

## trajectory.h
Based on the first two points of the old trajectory, the length and the speed are determined (line 28,29). and the direction for the new trajectory is taken in the form of the two points and the speed for the new spline construction is taken over (line 32-34).

The next four points are determined by averaging 3 A * points in the d-direction and speed-dependent points in the S direction (lines 44-81). These and all other A * points are back projected in Cartesian coordinates and translated as colons with velocity indication and relative S value into the corresponding output vector (line 88-99) to produce the splines (line 105-110). Finally, the waypoints are generated on them (line 112-136), but the maximum acceleration in the axial and lateral direction must be maintained.

For safety reasons, it is checked before a lane change, whether this is still free because A * can not spend any mobile trajectories due to the latency and the architecture. For this purpose, the method "Trajectory :: calc_gap" (line 139) is used.

## helpers.h
Here are the helpers, such as global variables and functions for A * including the conversion functions

## frenet.h
This object is a very reusable object. It reads all waypoints and describes the optional closed track with splines. The methods can convert Cartesian coordinates to Frenet coordinates and vice versa.
Source: The algorithms come from a post by Eric Lavigne "# t3-p-path-planning" of May 13, 2018.

## spline.h
simple cubic spline interpolation library without external dependencies
Source: https://kluge.in-chemnitz.de/opensource/spline/


