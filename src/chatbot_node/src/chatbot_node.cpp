#include "rclcpp/rclcpp.hpp"

#include "mrsd_msgs/msg/reply_msg.hpp"
#include "mrsd_msgs/msg/sent_msg.hpp"
#include <string>

using namespace std;

// Add your code here

int main(int argc, char **argv)
{

  ros::init(argc, argv, "chatbot_node");
  ros::NodeHandle n;

  // Add your code here

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}