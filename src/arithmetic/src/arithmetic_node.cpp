#include <string>
#include <algorithm>
#include <cctype>
#include <regex>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "mrsd_msgs/msg/sent_msg.hpp"
#include "mrsd_msgs/msg/arithmetic_reply.hpp"

using namespace std;
using namespace mrsd_msgs::msg;
using namespace rcl_interfaces::msg;
using std::placeholders::_1;

namespace mrsd
{
  class ArithmeticNode : public rclcpp::Node
  {
  public:
    ArithmeticNode() : Node("arithmetic_node")
    {
      // Put your publishers, subscribers, etc. here
      arithmetic_reply_pub = create_publisher<ArithmeticReply>("arithmetic_reply", 10);
      sent_msg_sub = create_subscription<SentMsg>("sent_msg", 10, std::bind(&ArithmeticNode::sentMsgCb, this, _1));
    }

    ~ArithmeticNode() {}

  private:
    rclcpp::Publisher<ArithmeticReply>::SharedPtr arithmetic_reply_pub;
    rclcpp::Subscription<SentMsg>::SharedPtr sent_msg_sub;

    void sentMsgCb(const SentMsg::SharedPtr msg)
    {

      string expr = msg->message;
      ArithmeticReply reply_msg;

      reply_msg.time_received = get_clock()->now().seconds();

      std::string segment;
      std::vector<float> seglist;

      std::stringstream ss(expr);

      string operator_string = regex_replace(expr, regex(R"([^*\/\-+])"), "");
      if (operator_string.length() > 1)
      {
        RCLCPP_ERROR(get_logger(), "Found multiple operators. Only one operator supported at a time.");
        return;
      }

      if (expr.find('+') != string::npos)
      {
        reply_msg.oper_type = reply_msg.ADD;
        while (std::getline(ss, segment, '+'))
        {
          seglist.push_back(stof(segment));
        }
        reply_msg.answer = seglist[0] + seglist[1];
      }
      else if (expr.find('-') != string::npos)
      {
        reply_msg.oper_type = reply_msg.SUBTRACT;
        while (std::getline(ss, segment, '-'))
        {
          seglist.push_back(stof(segment));
        }
        reply_msg.answer = seglist[0] - seglist[1];
      }
      else if (expr.find('*') != string::npos)
      {
        reply_msg.oper_type = reply_msg.MULTIPLY;
        while (std::getline(ss, segment, '*'))
        {
          seglist.push_back(stof(segment));
        }
        reply_msg.answer = seglist[0] * seglist[1];
      }
      else if (expr.find('/') != string::npos)
      {
        reply_msg.oper_type = reply_msg.DIVIDE;
        while (std::getline(ss, segment, '/'))
        {
          seglist.push_back(stof(segment));
        }
        if (seglist[1] == 0)
        {
          RCLCPP_ERROR(get_logger(), "Cannot divide by zero.");
          return;
        }
        reply_msg.answer = seglist[0] / seglist[1];
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Could not find operator.");
        return;
      }

      string equation = expr + " = " + to_string(reply_msg.answer);
      RCLCPP_INFO(get_logger(), equation.c_str());

      reply_msg.time_answered = get_clock()->now().seconds();
      reply_msg.process_time = reply_msg.time_answered - reply_msg.time_received;
      reply_msg.header.stamp = get_clock()->now();

      arithmetic_reply_pub->publish(reply_msg);
    }
  };

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrsd::ArithmeticNode>());
  rclcpp::shutdown();
  return 0;
}