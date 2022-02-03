# ROS Localisation and Navigation using Adaptive Monte Carlo Localisation (AMCL) with a skid-steer robot

This repository contains a Robot Operating System (ROS) implementation of a skid-steer robot model for uses with the ROS navigation stack.
The model uses the Adaptive Monte Carlo Localisation (AMCL) method for localisation with the Timed Elastic band planner method to navigate to goal locations. The original Elastic Band planner is not available in the latest editions of ROS however, so it was changed and is liable to change.

![skid-steer robot model](images/skid-steer-bot.png)

## Project website

This repository has an accompanying project page, contains the theory and details behind the code. It can be found [here](https://www.haidynmcleod.com/ros-skid-steer-navigation).

## Prerequisites

1. [Ubuntu](https://www.ubuntu.com/) OS or [debian](https://www.debian.org/distrib/)

2. Robot Operating System (ROS). Noetic is the preferred version and its installation instructions can be found [here](http://wiki.ros.org/ROS/Installation). 

3. Install ROS nodes required for the local and global planners, amcl, maps and motor control for the navigation stack.

```sh
sudo apt-get update
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-map-server
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-teb-local-planner
sudo apt-get install ros-noetic-global-planner
```

## Installing

Clone this repository in your catkin workspace 'src/' folder.

```sh
mkdir -p catkin_ws/src
cd ~/catkin_ws/src/
git clone https://github.com/grittED/simulation.git
```

Build the project:
```sh
cd ~/catkin_ws
catkin_make
```

If you haven’t already, the following line can be added to your .bashrc to auto-source all new terminals
This can be done by:
```sh
cd ~
vim .bashrc
```
Head to a clear space in the file and then insert (*i*) the following lines
```sh
source ~/catkin_ws/devel/setup.bash
source /opt/ros/noetic/setup.bash
```

Close the terminal, then when the next time you open the terminal (for the next steps), the above scripts will be automatically run.

## Run the Code

In a terminal window, type the following,
```sh
cd ~/catkin_ws
roslaunch skid_steer_bot udacity_world.launch
```

In a new terminal, run the 'amcl.launch' file.
```sh
cd ~/catkin_ws
source devel/setup.bash
roslaunch skid_steer_bot amcl.launch
```

At times, gazebo will struggle to launch properly, in such an event, try killing all existing gazebo processes.
```sh
killall gzserver
killall gzclient
```

Gazebo and Rviz will load and you should arrive at a result similar to the below.

![Gazebo & RViz with costmap](images/RvizGazebo.png)

###### Testing 

1. In Rviz, click on the 2D Nav Goal in the top menu. 
2. Click on the Rviz map where you want the robot to navigate too. 

You should arrive at a result similar to the below.

![navigation to a goal location](images/nav_goal.png)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
