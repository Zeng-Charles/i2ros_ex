#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>



//Class defining clients of supermarket
class mission_r2d2{

    //Node
    ros::NodeHandle nh_;

    //Timer that will run the main code
    ros::Timer timer_mission_r2d2_;

    //Coordinates
    float x_obj, y_obj, yaw_obj; // location of lost object

    //Modes
    int mission_flag; // 0 = getting ready, 1 = goint to object, 2 = picking up object, 3 = coming back

    //Velocity command message and publisher
    ros::Publisher vel_pub;
    geometry_msgs::Twist vel_msg;

    //Gripper command message and publisher
    ros::Publisher gripper_pub;
    std_msgs::Float64MultiArray gripper_msg;

    //Pick up publisher
    ros::Publisher pickup_pub;
    std_msgs::Bool pickup_msg;

public:

    //Constructor
    mission_r2d2()
    {
        //Set starting mission flag
        mission_flag = 1;

        //Declare velocity command publisher
        vel_pub = nh_.advertise<geometry_msgs::Twist>("/r2d2_diff_drive_controller/cmd_vel", 1);

        //Declare gripper command publisher
        gripper_pub = nh_.advertise<std_msgs::Float64MultiArray>("/r2d2_gripper_controller/command", 1);

        //Declare pickup message publisher
        pickup_pub = nh_.advertise<std_msgs::Bool>("/pickup_alert", 1);

        //Start timer running at 100Hz
        timer_mission_r2d2_ = nh_.createTimer(ros::Duration(0.01), &mission_r2d2::mission_r2d2_loop, this);

    }

    //////// MAIN LOOP ////////

    //Loop that runs at 100Hz (this is where the algorithm is located)
    void mission_r2d2_loop(const ros::TimerEvent& t){
        ROS_INFO_ONCE("Mission R2D2 started");

        if (mission_flag == 1){navigate_to_lost_object();}
        else if (mission_flag == 2){pick_up_object();}
        else if (mission_flag == 3){navigate_to_initial_pos();}
    }

    //////// FUNCTION - MISSION ////////

    //Navigate to position of lost object
    void navigate_to_lost_object()
    {
        ROS_INFO_ONCE("Status update--> 1- Navigating to lost object");
        
        //Get distance and yaw
        get_tf_pose(x_obj, y_obj, yaw_obj, "lost_object","base_link");
        float distance = sqrt(pow(x_obj, 2) + pow(y_obj, 2));
        float yaw = atan2(y_obj,x_obj);
        
        //Calculate velocity commands (FEEL FREE TO IMPROVE!)
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;        
        if (yaw_obj > 0.05){vel_msg.angular.z = -0.2;}  //rotate CCW
        if (yaw_obj < -0.05){vel_msg.angular.z = 0.2;}  //rotate CW  
        else                                            // already same orientation as object
        {
            if (distance > 1.0) {vel_msg.linear.x = 1;} //go straight
            else {mission_flag = 2;}                    //reached goal
        }
        vel_pub.publish(vel_msg);
    }

    //Pick up lost object
    void pick_up_object()
    {
        ROS_INFO_ONCE("Status update--> 2- Picking up lost object");

        //Close gripper
        gripper_msg.data.push_back(0.0); // 0.0 means close gripper
        gripper_msg.data.push_back(0.0); // 0.0 means close gripper
        gripper_msg.data.push_back(0.0); // 0.0 means close gripper

        gripper_pub.publish(gripper_msg);

        //Wait some time (2s) until the motion is completed
        ros::Duration(2).sleep();

        //Publish the message confirming pickup
        pickup_msg.data = true;
        pickup_pub.publish(pickup_msg);


        ROS_INFO("Pick up completed");

        //Change to nex mode
        mission_flag = 3;
    }

    // Return to initial postion
    void navigate_to_initial_pos()
    {
        ROS_INFO_ONCE("Status update--> 3- Navigating to initial position");

        //Get distance to starting point
        get_tf_pose(x_obj, y_obj, yaw_obj, "lost_object", "odom");
        float distance = sqrt(pow(x_obj, 2) + pow(y_obj, 2));

        //Navigate back
        if (distance > 1.0)                 //Send velocity commands
        {
            //Send velocity commands (backwards)
            vel_msg.linear.x = -1.0;
            vel_msg.angular.z = 0.0; 
            vel_pub.publish(vel_msg);
        }
        else                                //Check if arrived
        {
            ROS_INFO("Mission completed!");
            ros::shutdown();
        }
    }


    //////// FUNCTION - TOOLS ////////

    //Return coordinates of a frame respective to another 
    //Help: we send them as references, hence x and y member variables will be modified inside the function
    void get_tf_pose(   float& x, float& y, float& yaw,
                            const std::string frame1, const std::string frame2)
    {
        //Create a transform listener
        tf::TransformListener listener;
        tf::StampedTransform transform;
        bool transform_exception = 1;
        while (transform_exception)
        {
            try
            {
                listener.lookupTransform(frame1, frame2, ros::Time(0), transform);
                transform_exception = 0;

            }
            catch(tf::TransformException ex)
            {
                transform_exception = 1;
                continue;
            }
        }

        x = transform.getOrigin().x();
        y = transform.getOrigin().y();

        yaw = tf::getYaw(transform.getRotation());
    }

};


//Main function
int main(int argc, char **argv)
{
        // initialize node
        ros::init(argc, argv, "mission_r2d2");
        ros::NodeHandle nh_private("~");
        
        //initialize class
        mission_r2d2 R2D2;

        //Allow the timer to be called
        ros::spin();
}
        
        
