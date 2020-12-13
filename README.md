# Practical Course: Intelligent Mobile Robots with ROS (PCIMR)

## Tutorial 05: Path Planning & Control

### Introduction

This is the implement of *pcimr_tutorial_05 -- Path Planning & Control*.

In this tutorial, the task is to implement a package, which is supposed to find a path from start point to goal point using a star search.

The result of a star search is visualized in *rviz*.

![image](https://github.com/hoaklee/pcimr_tutorial/blob/pcimr_tutorial_05/resources/imgs/result_astar.png)

---
### Code Overview

The package `/pcimr_astar` was created and includes neccessary function of an a star search.

The nodes `/pcimr_astar` subscribes to 3 publishers and publishes 3 messages:

- Subscribers:
  - `/sub_pos`: *Point* from `/robot_pos`
  - `/sub_map`: *OccupancyGrid* from `/map`
  - `/sub_goal`: *PoseStamped* from `/move_base_simple/goal`
- Publisheres:
  - `/pub_path`: *Path* to `/global_path`
  - `/pub_plan`: *Marker* to `/visualization/plan`
  - `/pub_goal`: *Marker* to `/visualization/goal`

The simulation can be started by simply run:

    roslaunch pcimr_navigation navigation.launch

---
### Main idea

The main structrue of this algorithm can be divided into 5 parts:

  1. For each position of ego point, we first find the possible next points.
  2. Then calculate the heutistic cost `Node.h`, actual cost from start point to this point `Node.g` and the sum of them `Node.f`.
  3. After calculation, store all these points to a list called `open_list`.
  4. Find the points in `open_list` with minimal f value and store this point to `closed_list`.
  5. Start step *1* from the last point in `closed_list` until reaches the goal.
  
---
### Additional information

1. One thing important to start simulation: you have to select a goal through rviz yourself.
   - If the goal is not valid(in the boundary or out of range), you will get the information `Goal is not valid`.
   - If the goal is valid, the ego point(green point) will start moving.
