/**
* \file manual.cpp
* \brief Controller in manual modes 2 and 3
* \author Eléa Papin
* \version 1.0
* \date 12/03/2022
*
* \details
*
* Subscribes to: <BR>
* ° /current_mode
* ° /scan
* ° /manual_cmd_vel
*
* Publishes to: <BR>
* ° /cmd_vel
*
*
* Description :
* The manual node is there to handle the second and third modes. When the mode is set to the second, the twist generated
* by teleop_twist_keyboard is forwarded to the gazebo simulation without alterations. In the thrid mode, a correction is
* applied before publication of the twist so that the robot cannot drive into the walls. This correction is created thanks
* to the information provided by the laser scanner in the simulation
*
* 
**/


#include "ros/ros.h"
#include <string.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"

//The situation is the following : the mode 2 is completly manual, we just have to copy the commands established by
//teleop_twist_keyboard to the topic cmd_vel (remember, we remaped)
//mode 3 is assisted: correction before copy using laser scan infos


float f_lft; ///< Smallest value in the left forward region
float front; ///< Smallest value in the front forward region
float f_rgt; ///< Smallest value in the right forward region

// Current behavior
int current_mode = 0;  ///< User defined behavior mode

//Twist to be sent to gazebo
geometry_msgs::Twist my_twist; ///< Twist command to be published

ros::Publisher pub_twist;

//update the current mode using the user interface broadcast
/**
* \brief Retrieves the selected behavior mode
* \param msg is a std_msgs/Int32 argument
* \return Returns nothing
*
* This function is a call back for the subscription to the topic /current_mode. It retrieves a std_msgs/Int32 message.
* It updates the global variable current_mode.
*/
void ModeCallBack(const std_msgs::Int32 &msg){
    current_mode = msg.data;
}

//get user defined twist sent by teleop keyboard
/**
* \brief Retrieves the user defined twist
* \param msg is a geometry_msgs/Twist argument
* \return Returns nothing
*
* This function is a call back for the subscription to the topic /manual_cmd_vel. It retrieves a geometry_msgs/Twist message.
* It updates the global variable my_twist.
*/
void CmdCallBack(const geometry_msgs::Twist &msg)
{
    my_twist = msg;       
}

// Auxilliary function looking for the minimum in a given part of msg.ranges
/**
* \brief Search for a minimum in an array section
* \param msg is a sensor_msgs/LaserScan/ConstPtr argument
* \param i is an integer, index of the start of the section
* \param j is an integer, index of the end of the section, not included
* \return Returns nothing
*
* This is an auxilliary function computing the minimum of ranges within a given sector of a message provided
* by a laser scan. The minimum is the closest obstacle to the robot is the sector defined by indexes i and j.
*/
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

//mode 3: collisison avoidance assistance, to be called by the ScanCallBack
/**
* \brief Corrects user defined twist to avoid obstacles
* 
* \return Returns nothing, modifies a global variable
*
* Taking into account the minimum distance value in the sectors front_left,
* front an d front_right, it modifies the twist to clear the obstacles and avoid collisions.
*/
void Assistance()
{
    float safety_limit = 0.5;
    
    //if the robot is cmoser than the safety limit to the front wall
    // then we stop its forward motion
    if (front < safety_limit and my_twist.linear.x > 0.0)
    {
        my_twist.linear.x =  0.0;
        ROS_INFO("There is a wall ahead!");
    }
    // If the robot is too close to the left wall
    //  the robot is turned away from it
    if (f_lft < safety_limit)
    {
        my_twist.angular.z = 1.5; // go the other way
        ROS_INFO("You are too close to the left wall!");
    }
    //If the robot is not to close to the left but is to close to the right
    // it is turned away from the right wall.
    else if (f_rgt < safety_limit)
    {
        my_twist.angular.z = -1.5; // go the other way
        ROS_INFO("You are too close to the right wall!");
    }
    
}

//Callback for the LaserScan sensor
/**
* \brief Publishes a user defined twist in mode 2 and 3
* \param msg is an sensor_msgs/LaserScan/ConstPtr
* \return Returns nothing, publishes a twist
*
* Retrieve the sensor information from Laser scan on `/scan`, computes the closest obstacles.
* Call assistance if mode 3 is enabled, that used the obstacle infomation. Publishes a twist on `cmd_vel` in mode 2 and 3.
*/
void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg){

    //Treatment of the laser scan information only in mode 3
    if (current_mode == 3){
        int fifth;    
        fifth = (int)msg->ranges.size()/5; // we deal with positive values, i.e. / acts as the euclidean division
            
        //Retrieval of the smallest value in the three forward regions (i'm splitting ranges in 5)       
        f_lft = min_sector(msg, fifth +1 , 2*fifth);
        front = min_sector(msg, 2*fifth +1 , 3*fifth);
        f_rgt = min_sector(msg, 3*fifth +1 , 4*fifth);

        //Apply correction
        Assistance();
    }

    if (current_mode == 3 || current_mode ==2){
        //Publish updated twist to gazebo
         pub_twist.publish(my_twist);
     }
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
