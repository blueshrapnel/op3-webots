#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

class RobocupAction : public BT::TreeExecutionServer
{
public:
    RobocupAction(const rclcpp::NodeOptions &options) : TreeExecutionServer(options)
    {
        executeRegistration();
    }

    void onTreeCreated(BT::Tree &tree) override
    {
        logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
    }

    std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status,
                                                        bool was_cancelled) override
    {
        return std::nullopt;
    }

private:
    std::shared_ptr<BT::StdCoutLogger> logger_cout_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto action_server = std::make_shared<RobocupAction>(options);

    action_server->node()->declare_parameter<std::string>("behavior", "");
    std::string behavior = action_server->node()->get_parameter("behavior").as_string();
    if (!behavior.empty())
    {
        auto blackboard = BT::Blackboard::create();
        BT::Tree tree = action_server->factory().createTree(behavior, blackboard);
        bool finish = false;
        while (!finish && rclcpp::ok())
        {
            BT::NodeStatus status = tree.tickOnce();
            if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
                finish = true;
            tree.sleep(std::chrono::milliseconds(10));
        }
        RCLCPP_INFO(action_server->node()->get_logger(), "Predefined behavior '%s' finished with status: %s",
                    behavior.c_str(), BT::toStr(tree.rootNode()->status()).c_str());
    }

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                  std::chrono::milliseconds(250));
    exec.add_node(action_server->node());
    RCLCPP_INFO(action_server->node()->get_logger(), "BT action server has started. Waiting for behavior tree execution...");
    exec.spin();
    exec.remove_node(action_server->node());
    rclcpp::shutdown();
}
