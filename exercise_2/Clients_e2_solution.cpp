#include "ros/ros.h"
#include "std_msgs/String.h"

void customerCallback1(const std_msgs::String& msg){
        if (msg.data == "carrots")
        {
            //print in command window
            ROS_INFO("Customer 1: I got carrots!");
        }
}
void customerCallback2(const std_msgs::String& msg){
        if (msg.data == "tuna")
        {
            //print in command window
            ROS_INFO("Customer 2: I got tuna!");
        }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Clients");

  ros::NodeHandle n;

  ros::Subscriber customer1_sub = n.subscribe("veggies", 1000, customerCallback1);
  ros::Subscriber customer2_sub = n.subscribe("fish", 1000, customerCallback2);

  ros::spin();

  return 0;
}