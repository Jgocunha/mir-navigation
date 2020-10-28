#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* function declarations */
// bool moveToGoal(double xGoal, yGoal);

/* declare the coordinates of interest */
double xTopLeftCorner = 0.915;
double yTopLeftCorner = 19.139;

// bool moveToGoal(double xGoal, yGoal){
//     // ...

// }


int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    /* moving towards the goal*/

    goal.target_pose.pose.position.x =  xTopLeftCorner;
    goal.target_pose.pose.position.y =  yTopLeftCorner;
    goal.target_pose.pose.position.z =  0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("You have reached the destination");
    else
        ROS_INFO("The robot failed to reach the destination");

    return 0;
}