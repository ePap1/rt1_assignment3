/**
* \file user_interface.cpp
* \brief User interface implementing a the services server
* \author Eléa Papin
* \version 1.0
* \date 12/03/2022
*
* \details
*
* Publishes to: <BR>
* ° /current_mode
* ° /move_base_goal
*
* Services: <BR>
* ° /switch_mode
* ° /set_goal*
*
* Description :
* This node handles the two services with which the user can communicate with the simulation.
* It updates the running mode via /switch_mode and broadcast it to the manual_node on /current_mode.
* It also relays the goal request sent by the user on /set_goal to the action client using /move_base/goal.
*
* 
**/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string.h>

//services needed
#include "rt1_assignment3/BehaviorMode.h"
#include "rt1_assignment3/Goal.h"
#include <actionlib_msgs/GoalID.h>

//messages needed
#include "std_msgs/Int32.h"
#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>


//initialise the current mode to 0 : no mode selected
int current_mode = 0; ///< User defined behavior mode


//Prepare the goal to be sent
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; ///< Action client for move base
move_base_msgs::MoveBaseGoal goal;

bool goal_is_defined = false;


ros::Publisher pub_mode;
ros::Publisher pub_cancel;


//service to change mode
/**
* \brief implements the service /switch_mode.
* \param req is a rt1_assignment3/BehaviorMode/Request argument
* \param res is a rt1_assignment3/BehaviorMode/Response argument
* \return Returns true
*
* This service gets an integer corresponding to a desired behavior mode as request. This integer- must be 1, 2 or 3.
* When the mode is manual, aka 2 or 3, control instructions are provided
*/
bool switch_mode(rt1_assignment3::BehaviorMode::Request  &req, rt1_assignment3::BehaviorMode::Response &res){
   
    //The request is of type int32 (by definition of the service), there is no need to check the type
    if (req.mode >= 1 and req.mode <=3){
        current_mode = req.mode;
        res.success = true;
        ROS_INFO("The current mode is now %d ", current_mode);

        //Different message according to the mode
        if (current_mode ==2 or current_mode==3){
            printf("\nKeys to control the robot:\n"
                    "---------------------------\n"
                    "Moving around:\n"
                    "u    i    o\n"
                    "j    k    l\n"
                    "m    ,    .\n"

                    "q/z : increase/decrease max speeds by 10 percent\n"
                    "w/x : increase/decrease only linear speed by 10 percent\n"
                    "e/c : increase/decrease only angular speed by 10 percent\n"
                    "anything else : stop\n");
        }else{
            printf("\nTo set a goal, call the service /set_goal\n");
        }
    }
    //An invalid interger will no affect the system
    else{
        res.success = false;
        ROS_INFO("Please enter an existing mode, i.e. 1, 2 or 3");
    }
    
    return true;        
}


//service to set goal

/**
* \brief implements the service /set_goal.
* \param req is a rt1_assignment3/Goal/Request argument
* \param res is a rt1_assignment3/Goal/Response argument
* \return Returns true
*
* This service gets a couple of floats corresponding to a desired goal for the robot to reach.
* The fields of the global variable goal of move_base_msgs/MoveBaseGoal are filled there.
*/
bool set_goal(rt1_assignment3::Goal::Request  &req, rt1_assignment3::Goal::Response &res){
     goal_is_defined = false;
    
    //The goal is only to be considered if the mode 1 is active
    if (current_mode ==1){

        //update the new goal, regarless where this goal is in the map
        ROS_INFO("request is x=%f and y=%f", req.x, req.y);
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.orientation.w = 1.;
        goal.target_pose.pose.position.x = req.x;
        goal.target_pose.pose.position.y = req.y;
        res.success = true;
        ROS_INFO("The goal is now set to (%f, %f): ", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        
        goal_is_defined = true;
    }
    else{
        res.success = false;
        ROS_INFO("Goal definition is only for mode 1");
    }

    return true;
    
}


int main(int argc, char **argv)
{
    // Initialisation of the node and of its node handle
	ros::init(argc, argv, "user_interface");
    ros::NodeHandle nh;

    
    //services and publishers advertisment
    ros::ServiceServer service_mode = nh.advertiseService("/switch_mode", switch_mode);
    ros::ServiceServer service_goal = nh.advertiseService("/set_goal", set_goal);
    pub_cancel = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    pub_mode = nh.advertise<std_msgs::Int32>("/current_mode",1); 
    
    //action client constructor with the server name to connect to and set to automatically spin a thread
    MoveBaseClient ac("move_base", true);

    ros::Rate loop_rate(10);

    ROS_INFO("Current active mode: %d", current_mode);
    printf("\nThere are three available modes:\n"
            "1 - automatic planning and motion to a set goal\n"
            "2 - manual control\n"
            "3 - manual control with assitance to avoid collisions\n"
            "to switch, call the service /switch_mode\n"
            );

    while (ros::ok())
    {
        //Indicating the mode the the other nodes 
        std_msgs::Int32 msg_mode;
        msg_mode.data = current_mode;  
        pub_mode.publish(msg_mode);
        
        //Handling of automatic mode
        if(current_mode ==1 and goal_is_defined){

            //sending information to the action server
            ROS_INFO("Waiting for the server");
            ac.waitForServer();
            ROS_INFO("Sending goal");
            ac.sendGoal(goal);
            ROS_INFO("Goal sent");

            bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

            //If the action stopped before the timeout, check if the goal has been reached
            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("Action finished: %s",state.toString().c_str());
                            }
            //If the timeout stopped the action, the goal is cancelled
            else
            {
                ROS_INFO("The goal could not be reached in the alloted time : cancellation");
                auto msg = actionlib_msgs::GoalID();
                pub_cancel.publish(msg);
            }

            //In all cases, the goal has to be redefined
            goal_is_defined = false;   
        }
            
                
        ros::spinOnce();

        loop_rate.sleep();
        
    }

    return 0;    
  
}



