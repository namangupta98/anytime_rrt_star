# Anytime Rapidly-Growing Random Trees Star (RRT*)
Implementation of conference paper on Anytime Motion Planning using RRT* presesnted by Sertac Karaman, Matthew R. Walter, Alejandro Perez, Emilio Frazzoli and Seth Teller.

## Overview
The Rapidly-exploring Random Tree (RRT) algorithm, based on incremental sampling, efficiently computes motion plans. Although the RRT algorithm quickly produces candidate feasible solutions, it tends to converge to a solution that is far from optimal. Practical applications favor “anytime” algorithms that quickly identify an initial feasible plan, then, given more computation time available during plan execution, improve the plan toward an optimal solution. This paper describes an anytime algorithm based on the RRT∗ which (like the RRT) finds an initial feasible solution quickly, but (unlike the RRT) almost surely converges to an optimal solution. We present two key extensions to the RRT∗, committed trajectories and branch-and-bound tree adaptation, that together enable the algorithm to make more efficient use of computation time online, resulting in an anytime algorithm for real-time implementation.

## Run Instructions
- Navigate to your workspace and clone the repository.
```
cd ~/<ROS_Workspace>/src/
git clone https://github.com/namangupta98/anytime_rrt_star
```
- Build the package.
```
cd ~/<ROS_Workspace>
source devel/setup.bash
catkin_make
```
- Launch the gazebo world.
```
cd ~/<ROS_Workspace>
source devel/setup.bash
roslaunch anytime_rrt_star launcher.launch
```
- Open a new tab and run the RRT* Planner.
```
cd ~/<ROS_Workspace>
source devel/setup.bash
rosrun anytime_rrt_star controller.py
```
