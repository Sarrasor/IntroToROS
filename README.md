# IntroToROS

## Description

This repo contains simple URDF robot definition and control of it via C++ joint state publisher

Here is the Prober robot in Rviz:

![Prober in Rviz](images/prober.gif)

And here is the URDF graph for it:

![Prober URDF graph](images/prober_graph.png)

## How to run

- Add the `urdf_prober_robot` package to your `catkin_ws/src`
- Execute `catkin_make && source devel/setup.bash`
- Execute `roslaunch urdf_prober_robot create_robot.launch`

If you want to generate the URDF graph use those commands:

1. `rosrun xacro xacro robot.urdf > model.urdf`
2. `urdf_to_graphiz model.urdf`

`1` makes sure no Xacro is left in the robot.urdf and `2` produces the `model.pdf` file
