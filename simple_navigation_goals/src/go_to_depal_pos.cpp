#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal);
char choose();

/* declare the coordinates of interest */
double xfirstPos = -3.8;
double yfirstPos = -8.1;

double xsecondPos = -3.0;
double ysecondPos = -1.2;

double xthirdPos = -8.32;
double ythirdPos = 0.5;



bool goalReached = false;


char choose(){
	char choice='q';
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|Press a key:"<<std::endl;
	std::cout<<"|'0': Go to first depal position "<<std::endl;
	std::cout<<"|'1': Go to second depal position "<<std::endl;
	std::cout<<"|'2': Go to third depal position "<<std::endl;
	std::cout<<"|'q': Quit "<<std::endl;
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cin>>choice;

	return choice;
}



bool moveToGoal(double xGoal, double yGoal){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
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

   goal.target_pose.pose.position.x =  xGoal;
   goal.target_pose.pose.position.y =  yGoal;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the destination");
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the destination");
      return false;
   }

}



int main(int argc, char** argv){
   ros::init(argc, argv, "simple_navigation_goals");
   ros::NodeHandle n;
   ros::spinOnce();

   char choice = 'q';
   do{
      choice =choose();
      if (choice == '0'){
         goalReached = moveToGoal(xfirstPos, yfirstPos);
      }else if (choice == '1'){
         goalReached = moveToGoal(xsecondPos, ysecondPos);
      }else if (choice == '2'){
         goalReached = moveToGoal(xthirdPos, ythirdPos);
      }
      if (choice!='q'){
         if (goalReached){
            ROS_INFO("Position reached!");
            ros::spinOnce();

         }else{
            ROS_INFO("It was impossible to reach the target position!");
         }
      }
   }while(choice !='q');
   return 0;
}