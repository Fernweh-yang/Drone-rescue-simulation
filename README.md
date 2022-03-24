# Project -  Search and Rescue

## Introduction

The goal of the UAV is to fly over the avalanche and locate all the victims in a short time.

## How to Set up and Run the Project


### 1. Set up:

1. in folder /scr, add Submodule

   ```
   git submodule add https://github.com/ethz-asl/mav_trajectory_generation.git
   git submodule add https://github.com/ethz-asl/mav_comm.git
   git submodule add https://github.com/ethz-asl/eigen_catkin.git
   git submodule add https://github.com/ethz-asl/eigen_checks.git
   git submodule add https://github.com/ethz-asl/glog_catkin.git
   git submodule add https://github.com/ethz-asl/nlopt.git
   ```

2. in folder /src, load Submodule

   ```
   git submodule init
   git submodule update
   ```

3. in folder /project, build

   ```
   # only in first time build need to specify the Python version
   catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
   ```

### 2. Run the Project
If you want to run in Docker, please read [here](https://gitlab.lrz.de/ge23ged/autonomous-systems-2021-group-terminus/-/blob/main/project/Docker/README.md)  
If you want to run in your own Linux environment, please read below:  
We have two search models

- An intuitive idea, Inch-by-Inch Search 

  Read the [How to run inch-by-inch search](https://gitlab.lrz.de/ge23ged/autonomous-systems-2021-group-terminus/-/blob/main/project/src/README.md)

- An optimized Planning Method

  1. In Terminal 1, run unity

     ```
     roslaunch unity_bridge unity_sim.launch 
     ```

  2. In Terminal 2, run victim signal generation

     ```
     roslaunch victim_signal_gen victim.launch
     ```



  3. In Terminal 3, run rviz to see the trajectory in coordinate system

     ```
     roslaunch trajectory_visualization traj_visualize.launch 
     ```

  4. In Terminal 4, run trajectory planning (Please make sure it's the last step)

     ```
     rosrun planning planning_node
     ```


