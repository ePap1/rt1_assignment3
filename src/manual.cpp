
#include "ros/ros.h"
#include <string.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"

//The situation is the following : the mode 2 is completly manual, we just have to copy the commands established by
//teleop_twist_keyboard to the topic cmd_vel (remember, we remaped)
//mode 3 is assisted: correction before copy using laser scan infos


float f_lft;
float front;
float f_rgt;


int current_mode = 0;

geometry_msgs::Twist my_twist;

ros::Publisher pub_twist;

void ModeCallBack(const std_msgs::Int32 &msg){
    //update the current mode using the user interface broadcast
    current_mode = msg.data;
}


float min_sector(const sensor_msgs::LaserScan::ConstPtr& msg, int i, int j){
    float min = 100;
    float idx_min = i;

    for (int e=i; e<j; e++){
        if (msg->ranges[e]<min){
            min = msg->ranges[e];
            idx_min = e;
        }
    }
    return min;
}


void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg){
    //Treatment of the laser scan information only in mode 3
    if (current_mode == 3){
        int fifth;    
        fifth = (int)msg->ranges.size()/5; // we deal with positive values, i.e. / acts as the euclidean division
            
        //Retrieval of the smallest value in the three forward regions (i'm splitting ranges in 5)       
        f_lft = min_sector(msg, fifth +1 , 2*fifth);
        front = min_sector(msg, 2*fifth +1 , 3*fifth);
        f_rgt = min_sector(msg, 3*fifth +1 , 4*fifth);
    }
}

//mode 3: collisison avoidance assistance, to be called by the cmdCallBack
void Assistance()
{
    float safety_limit = 0.3;

    
    //if the robot is cmoser than the safety limit to the front wall
    // then we stop ist forward motion
    if (front < safety_limit)
    {
        my_twist.linear.x = 0.;
        ROS_INFO("There is a wall ahead: please turn!");
    }
    // In addition, the robot could be too close to the left wall
    // In which case, we turn the robot away from it
    if (f_lft < safety_limit)
    {
        my_twist.linear.x = 0.5*my_twist.linear.x;
        my_twist.angular.z = 0.5; // go the other way
        ROS_INFO("Danger, you are too close to the left wall!");
    }
    //If the robot is not to close to the left but is to close to the right
    // it is turned away from the right wall.
    else if (f_rgt < safety_limit)
    {
        my_twist.linear.x = 0.5*my_twist.linear.x;
        my_twist.angular.z = -0.5; // go the other way
        ROS_INFO("Do you want the robot to crash? Avoid right wall!");
    }
    
}

//get speed sent by teleop keyboard
void CmdCallBack(const geometry_msgs::Twist &msg)
{
    my_twist = msg;

    //In the mode 3, the user defined twist may need correcting to avoid obstacles
    if (current_mode ==3){
        Assistance();
    }

    pub_twist.publish(my_twist);
}


int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS system

    ros::init(argc, argv, "manual_node");
    ros::NodeHandle nh;    

    //initialize publisher and subscribers
    pub_twist = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub_mode = nh.subscribe("/current_mode",1, ModeCallBack);
    ros::Subscriber sub_scan = nh.subscribe("scan", 1, ScanCallBack);
    ros::Subscriber sub_twist = nh.subscribe("manual_cmd_vel", 1, CmdCallBack);

    
    ros::Rate loop_rate(10);

    while (ros::ok())
    {  
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}





/* 
Assignment 2 Assignment 2 Assignment 2 Assignment 2 Assignment 2
float min_sector(const sensor_msgs::LaserScan::ConstPtr& msg, int i, int j){
    float min = 100;
    float idx_min = i;

    for (int e=i; e<j; e++){
        if (msg->ranges[e]<min){
            min = msg->ranges[e];
            idx_min = e;
        }
    }
    return min;
}

Assignment 2 Assignment 2 Assignment 2 Assignment 2 Assignment 2
*/