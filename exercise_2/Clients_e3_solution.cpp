#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"



//Class defining clients of supermarket
class SupeROS_clients{

  //Declare ROS things
  ros::NodeHandle nh_;
  ros::Subscriber customer_sub_;
  ros::Publisher customer_pub_;

  //Declare customer identifier
  float customer_identifier_;

  //Declare string variables to store desired products
  std::string desired_product_;

public:

  //Constructor
  SupeROS_clients(float customer_identifier, std::string desired_product, std::string desired_topic)
  {
    //Store class identifier
    customer_identifier_ = customer_identifier;

    //Store desired product
    desired_product_ = desired_product;

    //Define subsciber and publisher
    customer_pub_ = nh_.advertise<std_msgs::Float64>("/Money", 1);
    customer_sub_ = nh_.subscribe(desired_topic, 1000, &SupeROS_clients::customerCallback, this);
  }

  //Callback where we check if the employee offers what customers wants and pay back
  void customerCallback(const std_msgs::String& msg){
        if (msg.data == desired_product_)
        {
                //pay back
                std_msgs::Float64 coin_msg;
                coin_msg.data = 1.0;
                customer_pub_.publish(coin_msg);

                //print in command window
                ROS_INFO("Customer %f: I got %s and I paid back!", customer_identifier_, desired_product_.c_str());
        }
  }
};

//Main function
int main(int argc, char **argv)
{
        // initialize node
        ros::init(argc, argv, "Clients");
        ros::NodeHandle nh_private("~");
        
        //initialize classes
        SupeROS_clients Clients_node1(1.0, "tuna","fish"), Clients_node2(2.0, "carrots","veggies");
        
        ros::spin();
}
