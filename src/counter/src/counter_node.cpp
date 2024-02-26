#include "rclcpp/rclcpp.hpp"

#include "mrsd_msgs/msg/reply_msg.hpp"
#include "mrsd_msgs/msg/sent_msg.hpp"

using namespace rclcpp;
using namespace mrsd_msgs::msg;
using std::placeholders::_1;

namespace mrsd
{
  class CounterNode : public rclcpp::Node
  {
  public:
    Subscription<ReplyMsg>::SharedPtr reply_msg_sub;
    Subscription<SentMsg>::SharedPtr sent_msg_sub;
    CounterNode() : Node("counter_node")
    {
      rclcpp::Time last_sent_msg_time;
      rclcpp::Time last_reply_msg_time;
      int num_reply_msg = 0;
      int num_sent_msg = 0;

      reply_msg_sub = this->create_subscription<ReplyMsg>("reply_msg", 10, std::bind(&CounterNode::replyMsgCb, this, _1));
      sent_msg_sub = this->create_subscription<SentMsg>("sent_msg", 10, std::bind(&CounterNode::sentMsgCb, this, _1));
    }

    ~CounterNode() {}

  private:
    void replyMsgCb(const ReplyMsg msg) const
    {
    }
    void sentMsgCb(const SentMsg msg) const
    {
    }
  };
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrsd::CounterNode>());
  rclcpp::shutdown();
  return 0;
}