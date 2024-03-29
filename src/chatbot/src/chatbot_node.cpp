#include <string>
#include <algorithm>
#include <cctype>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "mrsd_msgs/msg/reply_msg.hpp"
#include "mrsd_msgs/msg/sent_msg.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using namespace std;
using namespace mrsd_msgs::msg;
using namespace rcl_interfaces::msg;
using std::placeholders::_1;

namespace mrsd
{
  class ChatbotNode : public rclcpp::Node
  {
  public:
    ChatbotNode() : Node("chatbot_node")
    {
      // Put your publishers, subscribers, etc. here
      reply_msg_pub = create_publisher<ReplyMsg>("reply_msg", 10);
      sent_msg_sub = create_subscription<SentMsg>("sent_msg", 10, std::bind(&ChatbotNode::sentMsgCb, this, _1));
      declare_parameter("name", "John Doe");
    }

    ~ChatbotNode() {}

  private:
    rclcpp::Publisher<ReplyMsg>::SharedPtr reply_msg_pub;
    rclcpp::Subscription<SentMsg>::SharedPtr sent_msg_sub;

    void sentMsgCb(const SentMsg::SharedPtr msg)
    {
      RCLCPP_INFO(get_logger(), ("Q: " + msg->message).c_str());

      string processed_string = msg->message;
      ReplyMsg reply_msg;
      string name = get_parameter("name").as_string();

      // Keep only letters and spaces
      processed_string = regex_replace(processed_string, regex("[^A-Za-z]+"), "");

      // Convert to lowercase
      std::transform(
          processed_string.begin(), processed_string.end(), processed_string.begin(),
          [](unsigned char c)
          { return std::tolower(c); });

      if (
          processed_string == "hello" ||
          processed_string == "hey" ||
          processed_string == "hiya" ||
          processed_string == "heythere" ||
          processed_string == "hi" ||
          processed_string == "yo")
      {
        reply_msg.message = "Hello, " + name;
      }
      else if (
          processed_string == "whatisyourname" ||
          processed_string == "whatsyourname" ||
          processed_string == "whoareyou")
      {
        reply_msg.message = "My name is MRSD Siri.";
      }
      else if (
          processed_string == "howareyou" ||
          processed_string == "howsitgoing" ||
          processed_string == "whatsup" ||
          processed_string == "wassup")
      {
        reply_msg.message = "Feeling great with ROS2!";
      }
      else
      {
        RCLCPP_INFO(get_logger(), ("Message not understood. Ignoring." + reply_msg.message).c_str());
        return;
      }

      RCLCPP_INFO(get_logger(), ("A: " + reply_msg.message).c_str());

      reply_msg.header.stamp = get_clock()->now();
      reply_msg_pub->publish(reply_msg);
    }
  };

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrsd::ChatbotNode>());
  rclcpp::shutdown();
  return 0;
}