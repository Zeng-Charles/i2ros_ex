#include "ros/ros.h"
#include "std_msgs/String.h"

//callback for customer 1
void customerCallback1(const std_msgs::String& msg){
        if (...)
        {
            //print in command window
            ROS_INFO(...);
        }
}

//callback for customer 2
void customerCallback2(const std_msgs::String& msg){
        ...
}


//main function
int main(int argc, char **argv)
{

  ros::init(argc, argv, "Clients");

  ros::NodeHandle n;

  ros::Subscriber customer1_sub = ... //subscriber for customer 1
  ros::Subscriber ...   // subscrier for customer 2

  ros::spin();

  return 0;
}