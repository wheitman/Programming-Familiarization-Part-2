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

    void sentMsgCb(const SentMsg::SharedPtr msg) const
    {
      RCLCPP_INFO(get_logger(), msg->message.c_str());

      string processed_string = msg->message;
      ReplyMsg reply_msg;
      string name = get_parameter("name").as_string();

      // Keep only letters and spaces
      processed_string = regex_replace(processed_string, regex("[^A-Za-z ]+"), "");

      // Convert to lowercase
      std::transform(
          processed_string.begin(), processed_string.end(), processed_string.begin(),
          [](unsigned char c)
          { return std::tolower(c); });

      if (msg->message == "hello")
      {
        reply_msg.message = "Hello, " + name;
      }
      else if (msg->message == "what is your name")
      {
        reply_msg.message = "My name is MRSD Siri.";
      }
      else if (msg->message == "how are you")
      {
        reply_msg.message = "Feeling great with ROS2!";
      }
      else
      {
        reply_msg.message = "Hmm, I don't understand. I heard: " + processed_string;
      }

      // reply_msg.message = msg->message;
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