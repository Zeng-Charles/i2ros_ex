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

  //1- Declare publishers (employees) ///... TO BE COMPLETED ...///
  ros::Publisher MrFish = ;
  ros::Publisher MrVeggies = ;
  ros::Publisher MrFruits = ;


  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    // 2- Declare msg for publishing data (point 4 might be helpful)

    ///... TO BE COMPLETED ...///

    // 3- Define possible options for food ///... TO BE COMPLETED ...///
    std::vector<std::string> fisher_options = { "tuna", "salmon", "shark" };
    std::vector<std::string> veggies_options = { , ,};
    std::vector<std::string> fruit_options = { , ,};

    //4- Store current option

    ///... TO BE COMPLETED ...///
    
    if (count == 3){count = 0;} //reset counter if longer than amount of options
    msg_fisher.data = fisher_options[count];
    msg_veggies.data = ;
    msg_fruits.data = ;


    //5- Publish msg_X for all food types

    //... TO BE COMPLETED ...///

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}