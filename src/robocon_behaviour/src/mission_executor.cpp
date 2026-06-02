#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <string>

using namespace BT;

class WaitServiceTrigger : public BT::ActionNodeBase
{
public:
    WaitServiceTrigger(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::ActionNodeBase(name, config), node_(node), triggered_(false)
    {
        // Get service name from ports
        std::string service_name;
        if (!getInput("service_name", service_name)) {
            throw BT::RuntimeError("missing required input [service_name]");
        }

        // Create the service server
        srv_ = node_->create_service<std_srvs::srv::Trigger>(
            service_name,
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                RCLCPP_INFO(node_->get_logger(), "Trigger received on %s", srv_->get_service_name());
                triggered_ = true;
                response->success = true;
                response->message = "Triggered successfully";
            }
        );
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("service_name") };
    }

    BT::NodeStatus tick() override
    {
        if (triggered_) {
            triggered_ = false; // Reset for future ticks if needed
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void halt() override
    {
        triggered_ = false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
    bool triggered_;
};

class CallTriggerService : public BT::ActionNodeBase
{
public:
    CallTriggerService(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::ActionNodeBase(name, config), node_(node), request_sent_(false)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("service_name") };
    }

    BT::NodeStatus tick() override
    {
        std::string service_name;
        if (!getInput("service_name", service_name)) {
            throw BT::RuntimeError("missing required input [service_name]");
        }

        if (!client_ || client_->get_service_name() != service_name) {
            client_ = node_->create_client<std_srvs::srv::Trigger>(service_name);
        }

        if (!request_sent_) {
            if (!client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(node_->get_logger(), "Service %s not available", service_name.c_str());
                return BT::NodeStatus::FAILURE;
            }
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            future_ = client_->async_send_request(req).future.share();
            request_sent_ = true;
            return BT::NodeStatus::RUNNING;
        }

        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            request_sent_ = false;
            try {
                auto result = future_.get();
                if (result->success) {
                    RCLCPP_INFO(node_->get_logger(), "Successfully called %s: %s", service_name.c_str(), result->message.c_str());
                    return BT::NodeStatus::SUCCESS;
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "Service %s returned false", service_name.c_str());
                    return BT::NodeStatus::FAILURE;
                }
            } catch (...) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", service_name.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void halt() override
    {
        request_sent_ = false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> future_;
    bool request_sent_;
};

class MoveToLocation : public BT::ActionNodeBase
{
public:
    MoveToLocation(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::ActionNodeBase(name, config), node_(node), goal_sent_(false), goal_reached_(false)
    {
        publisher_ = node_->create_publisher<std_msgs::msg::String>("/planner/goal_location", 10);
        subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/controller/status", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    this->goal_reached_ = true;
                }
            });
    }

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("location_name")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!goal_sent_) {
            std::string loc_name;
            if (!getInput("location_name", loc_name)) {
                throw BT::RuntimeError("missing required input [location_name]");
            }

            std_msgs::msg::String goal_msg;
            goal_msg.data = loc_name;
            publisher_->publish(goal_msg);
            
            RCLCPP_INFO(node_->get_logger(), "Published location goal: %s", loc_name.c_str());
            
            goal_sent_ = true;
            goal_reached_ = false;
            return BT::NodeStatus::RUNNING;
        }

        if (goal_reached_) {
            RCLCPP_INFO(node_->get_logger(), "Controller reported goal reached!");
            goal_sent_ = false; // Reset for future ticks
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void halt() override
    {
        goal_sent_ = false;
        goal_reached_ = false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
    bool goal_sent_;
    bool goal_reached_;
};

class DetectKFS : public BT::ActionNodeBase
{
public:
    DetectKFS(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::ActionNodeBase(name, config), node_(node), request_sent_(false)
    {
        client_ = node_->create_client<std_srvs::srv::Trigger>("/detect_center_kfs");
    }

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("kfs_type") };
    }

    BT::NodeStatus tick() override
    {
        if (!request_sent_) {
            if (!client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(node_->get_logger(), "Service /detect_center_kfs not available");
                return BT::NodeStatus::FAILURE;
            }
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            future_ = client_->async_send_request(req).future.share();
            request_sent_ = true;
            return BT::NodeStatus::RUNNING;
        }

        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            request_sent_ = false;
            try {
                auto result = future_.get();
                setOutput("kfs_type", result->message);
                RCLCPP_INFO(node_->get_logger(), "Detected KFS: %s", result->message.c_str());
                return BT::NodeStatus::SUCCESS;
            } catch (...) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to call detection service");
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void halt() override
    {
        request_sent_ = false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> future_;
    bool request_sent_;
};

class CheckString : public BT::ConditionNode
{
public:
    CheckString(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("value"),
            BT::InputPort<std::string>("target")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string value, target;
        if (!getInput("value", value) || !getInput("target", target)) {
            return BT::NodeStatus::FAILURE;
        }
        return (value == target) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class IncrementCount : public BT::SyncActionNode
{
public:
    IncrementCount(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::BidirectionalPort<int>("count") };
    }

    BT::NodeStatus tick() override
    {
        int count = 0;
        if (!getInput("count", count)) {
            count = 0;
        }
        count++;
        setOutput("count", count);
        return BT::NodeStatus::SUCCESS;
    }
};

class CheckCount : public BT::ConditionNode
{
public:
    CheckCount(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<int>("count"),
            BT::InputPort<int>("target")
        };
    }

    BT::NodeStatus tick() override
    {
        int count = 0, target = 0;
        if (!getInput("count", count) || !getInput("target", target)) {
            return BT::NodeStatus::FAILURE;
        }
        return (count < target) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class WaitDuration : public BT::StatefulActionNode
{
public:
    WaitDuration(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("sec") };
    }

    BT::NodeStatus onStart() override
    {
        int sec = 0;
        if (!getInput("sec", sec)) {
            return BT::NodeStatus::FAILURE;
        }
        start_time_ = std::chrono::steady_clock::now();
        wait_time_ = std::chrono::seconds(sec);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (std::chrono::steady_clock::now() - start_time_ >= wait_time_) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
    std::chrono::seconds wait_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mission_executor");

    BT::BehaviorTreeFactory factory;

    // Register our custom nodes
    factory.registerBuilder<WaitServiceTrigger>("WaitServiceTrigger", 
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<WaitServiceTrigger>(name, config, node);
        });

    factory.registerBuilder<CallTriggerService>("CallTriggerService", 
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<CallTriggerService>(name, config, node);
        });

    factory.registerBuilder<MoveToLocation>("MoveToLocation", 
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<MoveToLocation>(name, config, node);
        });

    factory.registerBuilder<DetectKFS>("DetectKFS", 
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<DetectKFS>(name, config, node);
        });

    factory.registerNodeType<CheckString>("CheckString");
    factory.registerNodeType<WaitDuration>("WaitDuration");
    factory.registerNodeType<IncrementCount>("IncrementCount");
    factory.registerNodeType<CheckCount>("CheckCount");

    // Load the XML file
    std::string xml_file = "/home/robot/robocon_ws/src/robocon_behaviour/behavior_trees/mission.xml";
    node->declare_parameter("bt_xml", xml_file);
    xml_file = node->get_parameter("bt_xml").as_string();

    auto tree = factory.createTreeFromFile(xml_file);

    rclcpp::Rate rate(10); // 10 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        
        BT::NodeStatus status = tree.tickRoot();
        
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Mission Completed Successfully!");
            break;
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(node->get_logger(), "Mission Failed!");
            break;
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
