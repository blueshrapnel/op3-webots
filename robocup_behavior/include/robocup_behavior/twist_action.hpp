#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace BT;

class TwistAction : public RosTopicPubNode<geometry_msgs::msg::Twist>
{
public:
  TwistAction(const std::string &name, const NodeConfig &conf,
              const RosNodeParams &params)
      : RosTopicPubNode<geometry_msgs::msg::Twist>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<float>("x"), InputPort<float>("y"), InputPort<float>("yaw")});
  }

  bool setMessage(geometry_msgs::msg::Twist &msg) override;
};
