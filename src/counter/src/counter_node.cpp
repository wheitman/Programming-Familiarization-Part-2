#include "rclcpp/rclcpp.hpp"

#include "mrsd_msgs/msg/reply_msg.hpp"
#include "mrsd_msgs/msg/sent_msg.hpp"
#include "mrsd_msgs/srv/counter.hpp"

using namespace rclcpp;
using namespace mrsd_msgs::msg;
using namespace mrsd_msgs::srv;
using std::placeholders::_1;
using std::placeholders::_2;

namespace mrsd
{
  class CounterNode : public rclcpp::Node
  {
  public:
    Subscription<ReplyMsg>::SharedPtr reply_msg_sub;
    Subscription<SentMsg>::SharedPtr sent_msg_sub;
    Service<Counter>::SharedPtr counter_service;
    CounterNode() : Node("counter_node")
    {
      num_sent_msg = 0;
      num_reply_msg = 0;
      reply_msg_sub = this->create_subscription<ReplyMsg>("reply_msg", 10, std::bind(&CounterNode::replyMsgCb, this, _1));
      sent_msg_sub = this->create_subscription<SentMsg>("sent_msg", 10, std::bind(&CounterNode::sentMsgCb, this, _1));

      counter_service = this->create_service<Counter>("message_counter", std::bind(&CounterNode::counterServiceCb, this, _1, _2));
    }

    ~CounterNode() {}

  private:
    int num_reply_msg;
    int num_sent_msg;
    rclcpp::Time last_sent_msg_time;
    rclcpp::Time last_reply_msg_time;
    void replyMsgCb(const ReplyMsg msg)
    {
      num_reply_msg++;
      last_reply_msg_time = msg.header.stamp;
    }
    void sentMsgCb(const SentMsg msg)
    {
      num_sent_msg++;
      last_sent_msg_time = msg.header.stamp;
    }

    float getElapsedTime(rclcpp::Time time)
    {
      rclcpp::Time now = this->get_clock()->now();
      double now_sec = now.seconds();
      double then_sec = time.seconds();
      return now_sec - then_sec;
    }

    void counterServiceCb(const Counter::Request::SharedPtr request,
                          Counter::Response::SharedPtr response)
    {
      RCLCPP_INFO(get_logger(), "Got a request!");

      switch (request->req_id)
      {
      case 0:
        RCLCPP_INFO(get_logger(), "Total message count");
        response->reply = num_sent_msg + num_reply_msg;
        break;
      case 1:
        RCLCPP_INFO(get_logger(), "Total replied messages");
        response->reply = num_reply_msg;

        break;
      case 2:
        RCLCPP_INFO(get_logger(), "Total sent messages");
        response->reply = num_sent_msg;
        break;
      case 3:
        RCLCPP_INFO(get_logger(), "Time since last reply");
        response->reply = getElapsedTime(last_reply_msg_time);
        break;

      case 4:
        RCLCPP_INFO(get_logger(), "Time since last user message");
        response->reply = getElapsedTime(last_sent_msg_time);
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Invalid req_id!");
      }
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