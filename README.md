# rt1_assignment3

## Introduction
This package is an answer to the third assignement for the Research Track 1 class, for the first year of the Robotics Engineering degree at UniGe, in 2021-2022. The assignment consists in writing a ROS package to choose and control the motion of a robot in a given simulated environment.

The environment is a could be assimilated to a buidling floor plan where walls create a surrounding barriere and some inner obstacles. The robot must stay clear of the walls, and to do so, it is equiped with a laser scanner. To control the robot, the user can either choose to set a goal and let a planning node set the appropriate or he can define the twist with the keyboard. The user should use services to choose the control mode and to set goals.

## How to run
### Installation
The simulator requires ROS as well as several packages. You have to clone the following repositories and install the packages teleop_twist_keyboard and navigation. In your ROS workspace's src/ folder, execute the commands:
```Shell
git clone -b <branch-for-your-distro> https://www.github.com/CarmineD8/final_assignment
git clone https://www.github.com/CarmineD8/slam_gmapping
sudo apt-get update
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-teleop-twist-keyboard
```
And of course, to test my code, you will need to clone this repository
```Shell
git clone https://github.com/ePap1/rt1_assignment3
```
To compile the packages in your ros workspace, use
```Shell
catkin_make
```

### Execution
To run the simulation, execute the following commands (in different shell windows)
```Shell
roscore
roslaunch rt1_assignment3 rt1_a3_sim
roslaunch rt1_assignment3 rt1_a3
```


### Usage
Once the system is running, the user can interact with the simulation using two services, to be called using:
```Shell
rosservice call <name_of_service> <request>
```
The available services are
- /switch_mode : change the running mode between 1 - automatic, 2 - manual and 3 - manual with assistance
- /set_goal : in mode 1, defines the goal to be reach (pair of float)

To write the request, once you have written rosservice call <name_of_service>, press twice the tab key to have the message template. You can then fill it with the values you are interested in.

## Full documentation
Get the full [documentation](https://epap1.github.io/rt1_assignment3/) here.

## ui_node

This node handles the two services with which the user can communicate with the simulation. It updates the running mode via `/switch_mode` and broadcast it to the manual_node on `/current_mode`. It also relays the goal request sent by the user on `/set_goal` to the action client using `/move_base/goal`.

### functions

- bool switch_mode(rt1_assignment3::BehaviorMode::Request  &req, rt1_assignment3::BehaviorMode::Response &res) : implements the service /switch_mode.
- bool set_goal(rt1_assignment3::Goal::Request  &req, rt1_assignment3::Goal::Response &res) : implements the service /set_goal
- main : initialise all elements : node, node handle, publisher and servers. It also communicate with the action client to send goals.

### Pseudo-code
main
```Shell
    while ros runs smoothly
        publish current mode
        if (in mode one) and (goal_is_defined) then
            wait for action client server
            send goal 
            if action ended before timeout then
                get action status
                print action status 
            else
                print failure to reach goal
            end if
        reset goal_is_defined
        end if
    end if
```


## manual_node
The manual node is there to handle the second and third modes. It listens to `/current_mode` continuously and when on mode 2 or 3, it publishes a twist on  the topic `/cmd_vel` which modifies the behaviour of the robot in the simulation. It also listens to `/scan` to retrieve obstacle information from gazebo. In mode 2, the laser scan information is discarded and the user defined twist published onto `/manual_cmd_vel` is directly published. In mode 3, a correction is performed before publication.

### functions

- void ModeCallBack(const std_msgs::Int32 &msg) : updates the global varaible current_mode from topic `/current_mode`.
- void CmdCallBack(const geometry_msgs::Twist &msg) : updates the global varaible my_twist from topic `/manual_cmd_vel`.
- float min_sector(const sensor_msgs::LaserScan::ConstPtr& msg, int i, int j) : Auxilliary function computing the minimum of ranges within a given sector.
- void Assistance() : Taking into account the minimum distance value in the sectors front_left, front an d front_right, it modifies the twist to clear the obstacles and avoid collisions.
- void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) : Retrieve the sensor information from Laser scan on `/scan`, call assistance if necessary, then publish a twist on `cmd_vel`.
- main : initialise all elements : node, node handle, publisher and subscribers.

## flowchart of the communication
[![](https://mermaid.ink/img/pako:eNp1UbuOwyAQ_BVE5ZMSuXdxkl-6Kk0ezckSWsMmQTFgYZwoZ_vfD19wdCkCDTszuzvsDpQbgTShJwvtmeyLShN_0ujQoSVSO7RH4PjxgLNoA7qHhnCjnTVNgPNoY65IaugWYRHtsUHTMneTnWMXvNcGrAhsGX3BD9YmhN_DYVdup0qHkKzXn2PcoWMnA81I0lf8Jh0_M-VtP6l0psgY895a1C6Q2SupvEc2e4w5aI6-cP5O8Ogb6HwprgS7zmllGMYbvAx459uMwUOxtPgbH3umZP--RsZlTCMp5mHMl66oQqtACr-jYVZX1J1RYUUT_xRgLxWt9OR1fSvAYSmkM5YmR2g6XFHondndNaeJsz0uokKC37cKqukX-wuilg)](https://mermaid-js.github.io/mermaid-live-editor/edit#pako:eNp1UbuOwyAQ_BVE5ZMSuXdxkl-6Kk0ezckSWsMmQTFgYZwoZ_vfD19wdCkCDTszuzvsDpQbgTShJwvtmeyLShN_0ujQoSVSO7RH4PjxgLNoA7qHhnCjnTVNgPNoY65IaugWYRHtsUHTMneTnWMXvNcGrAhsGX3BD9YmhN_DYVdup0qHkKzXn2PcoWMnA81I0lf8Jh0_M-VtP6l0psgY895a1C6Q2SupvEc2e4w5aI6-cP5O8Ogb6HwprgS7zmllGMYbvAx459uMwUOxtPgbH3umZP--RsZlTCMp5mHMl66oQqtACr-jYVZX1J1RYUUT_xRgLxWt9OR1fSvAYSmkM5YmR2g6XFHondndNaeJsz0uokKC37cKqukX-wuilg)



## Possible improvments
The user experience could be better if a window was created to call the different services. The speed at which the robot reaches the goal is quite slow right now, but it may be due to the computation time.
