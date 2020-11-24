# Practical Course: Intelligent Mobile Robots with ROS (PCIMR)

## Tutorial 02: Robotics hardware & kinematics

### Introduction

The task of this second tutorial is to control the velocity according to obstacles in the direction of movment.

It can stop when the obstacle distance is less than 0.4m and will slow down when the distance is smaller than 1.2m using propotional velocity:
- If you want to change the default parameter, just change it in the launch file /launch/velocity_controller.launch

But there are still two problems when I'm doing experiment:
1. Sometimes the algorithm does not update according to the established frequency. 
    - For example, If I set the frequency to 1Hz and to see running of the algorithm by output linear.x velocity every step. The velocity supposed to appear every second. But sometimes it might stop update for a few seconds and the robot is still moving. In this case, if the robot is supposed to stop in this period, it will not stop and hit the wall. And it is not a problem in the algorithm, more like problem in gazebo or ROS?
2. For omidirctional robot rto-1, for example, when it move towards the obstacle at 45 degree, if the y velocity reaches 0 and x velocity is still there, it will keep moving in x direction, which is inconsist with the original direction. I'm still thinking about improve the algorithm to solve it.

### Code Overview

The process of the algorithm it is as fllowing:
- receive input comman from teleop keyboard
- determine if this command is feasible in current situation
- pass adjusted command to robot

In order to accomplish the mentioned functions, I create a package called pcimr_tutorial_02. It contains a subscriber and a publisher:
1. subscribes input/cmd_vel(here we remap the topicname of teleop keyboard publisher from /cmd_vel to /input/cmd_vel in order to prevent chaos)
2. publishes command to /pioneer/cmd_vel if the robot is p3dx or to /cma_vel if the robot is rto-1

Launch the simulation by running:

    roslaunch pcimr_tutorial_02 velocity_controller.launch

Or if you want to use p3dx robot, you have to run the following first:

    export ROBOT=p3dx

### Questions

1. Does your controller work out of the box? Can you hit walls?
  The controller works out of box. It will not hit the wall if the system is updating at supposed frequency.
2. Update your controller so that it works with both, differential and omnidirectional drive.
- Since omidirectional robot can move in every direction, the velocity consists of x and y velocity. So additional constrain in y direcition should be add to avoid collision in y direction.
- Another problem is that the sensor can only detect a range of 230 degree, It can hardly see the situation in the back, so I just set the backward velocity to 0, to prevent collision.
