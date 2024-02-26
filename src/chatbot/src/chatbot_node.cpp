#include "rclcpp/rclcpp.hpp"

#include "mrsd_msgs/msg/reply_msg.hpp"
#include "mrsd_msgs/msg/sent_msg.hpp"
#include <string>

using namespace std;

namespace mrsd
{
  class ChatbotNode : public rclcpp::Node
  {
  public:
    ChatbotNode() : Node("chatbot_node")
    {
      // Put your publishers, subscribers, etc. here
    }

    ~ChatbotNode() {}
  };
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrsd::ChatbotNode>());
  rclcpp::shutdown();
  return 0;
}