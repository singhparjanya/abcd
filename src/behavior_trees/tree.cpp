#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <condition_variable>

std::string shared_goal_command;
std::mutex goal_mutex;

class GoalPublisherNode : public BT::SyncActionNode
{
public:
    GoalPublisherNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("goal_publisher"))
    {
        publisher_ = node_->create_publisher<std_msgs::msg::String>("final_goal", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(goal_mutex);

        if (shared_goal_command.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "No goal received from goal_command topic.");
            return BT::NodeStatus::FAILURE;
        }

        auto msg = std_msgs::msg::String();
        msg.data = shared_goal_command;
        publisher_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), "Published goal to final_goal topic: %s", msg.data.c_str());
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class ChargingStationPublisherNode : public BT::SyncActionNode
{
public:
    ChargingStationPublisherNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("charging_station_publisher"))
    {
        publisher_ = node_->create_publisher<std_msgs::msg::String>("final_goal", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        auto msg = std_msgs::msg::String();
        msg.data = "tb1 1.0 2.0 0.0"; // Coordinates for the charging station
        publisher_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), "Published charging station coordinates to final_goal topic.");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class CheckBattery : public BT::SyncActionNode
{
public:
    CheckBattery(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared("check_battery_node"))
    {
        battery_level_ = 100; // Hardcoded for now, replace with actual battery level checking
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        if (battery_level_ > 30)
        {
            RCLCPP_INFO(node_->get_logger(), "Battery level is sufficient: %d%%", battery_level_);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Battery level is low: %d%%", battery_level_);
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    int battery_level_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<CheckBattery>("CheckBattery");
    factory.registerNodeType<GoalPublisherNode>("GoalPublisher");
    factory.registerNodeType<ChargingStationPublisherNode>("ChargingStationPublisher");

    auto tree = factory.createTreeFromFile("/home/cas/robot_ws/src/turtlebot3_multi_robot/src/behavior_trees/tree.xml");

    rclcpp::executors::SingleThreadedExecutor executor;
    auto bt_node = rclcpp::Node::make_shared("bt_node");

    std::mutex mutex;
    std::condition_variable cv;
    bool goal_received = false;

    auto subscription = bt_node->create_subscription<std_msgs::msg::String>(
        "goal_command", 10,
        [&](std_msgs::msg::String::SharedPtr msg) {
            {
                std::lock_guard<std::mutex> lock(goal_mutex);
                shared_goal_command = msg->data;
            }
            {
                std::lock_guard<std::mutex> lock(mutex);
                goal_received = true;
                cv.notify_all();
                RCLCPP_INFO(bt_node->get_logger(), "Received goal command: %s", msg->data.c_str());
            }
        });

    // Add the node to the executor
    executor.add_node(bt_node);

    // Create a new thread for behavior tree processing
    std::thread bt_thread([&]() {
        while (rclcpp::ok())
        {
            // Wait for a goal to be received before starting the tree execution
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&]() { return goal_received || !rclcpp::ok(); });

            // Break if ROS is shutting down
            if (!rclcpp::ok())
            {
                break;
            }

            // Tick the behavior tree
            tree.tickRoot();
            goal_received = false;
        }
    });

    // Spin the executor in the main thread
    executor.spin();

    // Wait for the behavior tree thread to finish
    if (bt_thread.joinable())
    {
        bt_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}

