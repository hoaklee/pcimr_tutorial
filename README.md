# Practical Course: Intelligent Mobile Robots with ROS (PCIMR)

## Tutorial 03: Probability Theory & Localization

### Introduction

This is the implement of *pcimr_tutorial_03* -- *Probability Theory & Localization*.

In this tutorial, the task is to using particle filter to localize ego point and publish a probability map to visualize the probability.

To visualize the probability, you can simply choose topic `/robot_pos_map` for `Robot Pose Probability` and choose color scheme `costmap`

The reusult of algorithm can be seen in the picture below:

---
### Code Overview

To implement localization function, I create a package `/pcimr_localization` package, which includes necessary functions of doing localization.

The node subscribes to 4 publishers and pulishes 3 messages:
- Subscribers:
  - `/map_sub`: OccupancyGrid from `/map`
  - `/sensor_sub`: LaserScan from `/scan`
  - `/move_sub`: String from `/move`
  - `/goal_sub`: Marker from `/visualization/goal`
- Publishers:
  - `/point_pub`: Point to `/robot_pos`
  - `/marker_pub`: Marker to `/visualization/robot_pos`
  - `/grid_pub`: OccupancyGrid to `/robot_pos_map`

The simulation can be started by running:

    roslaunch pcimr_localization pcimr_localization.launch

#### Main idea:
- The probability of the ego point's location is calculated by multiply the *sensor probability map* and *move probability map*. After calculating the probability map of ego piont, choose the most likely position, and publish this position to *navigation node*.

#### Move command:

The move command is modified to be uncertain. There are 5 values specifying this uncertainty:

    [𝑝(𝑢=𝑢 ̂ ),𝑝(𝑢=𝑢 ̂−90°),𝑝(𝑢=𝑢 ̂+90°),𝑝(𝑢=𝑢 ̂−180°), 𝑝(𝑢=∅)]

where 𝑢 is the performed action, 𝑢 ̂ is the expected action, 𝑢 ̂−90°, 𝑢 ̂+90°,  𝑢 ̂−180° the actions left, right and backwards with respect to the expected action 𝑢 ̂. The last one, ∅, stands in this case for no movement at all, i.e. the robot stays at its position.

Its default value is *[0.9, 0.04, 0.04, 0, 0.02]*, if you want to change the value, two parameters have to be modified:
- In the launch file *navigation.launch* of *pcimr_navigation* package.
- In *probability_location.py*, parameter *self.move_prob*.
#### Initilizing point:

By default, once the ego point reaches the goal, a new goal point will be assigned and the ego point will start moving from the previous goal point.
  
If you want the ego point to start moving at a random point, there are two parameters which have to be modified:
- set the value of *rand_ini_pos* in *navigation.launch* to True.
- set the value of *self.random* in *probability_location.py* to True.

#### Additional information:
   
1. Set move probability to *[0.7,  0.1,  0.1,  0.0,  0.1]*:
In this case, the moving uncertainty is increased, the algorithm still manages to localize, but will correct position more often due to higher uncertainty.
   
2. Set the param. *rand_ini_pos* to true:
The algorithm cannot deal with this problem.
To deal with it, I add a subscriber that get the goal position. Once ego piont reaches the goal point, the probability map will be equal to sensor probability map, so it will get an estimate position according to sensor information first.

