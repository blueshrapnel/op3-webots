#include "robocup_behavior/twist_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool TwistAction::setMessage(geometry_msgs::msg::Twist &msg)
{
    // Retrieve input values from the behavior tree ports.
    float x, y, yaw;
    if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get input values for TwistAction");
        return false;
    }
    // Set the Twist message values.
    msg.linear.x = x;
    msg.linear.y = y;
    msg.angular.z = yaw;
    RCLCPP_DEBUG(node_->get_logger(), "TwistAction setMessage: x=%f, y=%f, yaw=%f", x, y, yaw);

    return true;
}

// Plugin registration.
BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory &factory, BT::RosNodeParams &params)
{
    if (params.default_port_value.empty())
        params.default_port_value = "/cmd_vel";
    factory.registerNodeType<TwistAction>("Twist", params);
}
