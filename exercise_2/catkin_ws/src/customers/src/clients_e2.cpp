#include "ros/ros.h"
#include "std_msgs/String.h"

//callback for customer 1
void customerCallback1(const std_msgs::String& msg){
        if (msg.data == "carrots")
        {
            //print in command window
            ROS_INFO("I purchased %s", msg.data.c_str());
        }
}

//callback for customer 2
void customerCallback2(const std_msgs::String& msg){
        if (msg.data == "tuna")
        {
            //print in command window
            ROS_INFO("I purchased %s", msg.data.c_str());
        }
}


//main function
int main(int argc, char **argv)
{

  ros::init(argc, argv, "Clients");

  ros::NodeHandle n;

  ros::Subscriber customer1_sub = n.subscribe("veggies", 1000, customerCallback1); //subscriber for customer 1
  ros::Subscriber customer2_sub = n.subscribe("fisher", 1000, customerCallback2);   // subscrier for customer 2

  ros::spin();

  return 0;
}