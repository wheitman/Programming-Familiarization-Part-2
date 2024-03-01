#include "rclcpp/rclcpp.hpp"

#include "mrsd_msgs/msg/reply_msg.hpp"
#include "mrsd_msgs/msg/sent_msg.hpp"
#include <string>

using namespace std;
using namespace mrsd_msgs::msg;
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
    }

    ~ChatbotNode() {}

  private:
    rclcpp::Publisher<ReplyMsg>::SharedPtr reply_msg_pub;
    rclcpp::Subscription<SentMsg>::SharedPtr sent_msg_sub;

    void sentMsgCb(const SentMsg::SharedPtr msg) const
    {
      RCLCPP_INFO(get_logger(), msg->message.c_str());

      ReplyMsg reply_msg;
      reply_msg.message = msg->message;
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