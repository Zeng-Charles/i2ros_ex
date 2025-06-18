#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//main function
int main(int argc, char **argv)
{

  //start node
  ros::init(argc, argv, "SupeROS");

  //create variable for node
  ros::NodeHandle n;

  //declare publishers (employees)
  ros::Publisher MrFish = n.advertise<std_msgs::String>("fish", 1000);
  ros::Publisher MrVeggies = n.advertise<std_msgs::String>("veggies", 1000);
  ros::Publisher MrFruits = n.advertise<std_msgs::String>("fruits", 1000);


  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    // declare msg for publishing data
    std_msgs::String msg_fisher;
    std_msgs::String msg_veggies;
    std_msgs::String msg_fruits;

    //define possible options for food
    std::vector<std::string> fisher_options = { "tuna", "salmon", "shark" };
    std::vector<std::string> veggies_options = { "onion", "potatoes", "carrots" };
    std::vector<std::string> fruit_options = { "bananas", "apples", "grapes" };

    //store current option
    if (count == 3){count = 0;} //reset counter if longer than amount of options
    msg_fisher.data = fisher_options[count];
    msg_veggies.data = veggies_options[count];
    msg_fruits.data = fruit_options[count];


    //publish
    MrFish.publish(msg_fisher);
    MrVeggies.publish(msg_veggies);
    MrFruits.publish(msg_fruits);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}