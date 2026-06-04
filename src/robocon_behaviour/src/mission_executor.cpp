#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <string>

using namespace BT;

#include <robocon_interfaces/srv/get_high_level_plan.hpp>
#include <robocon_interfaces/srv/get_low_level_plan.hpp>
#include <robocon_interfaces/srv/pick_kfs.hpp>
#include <robocon_interfaces/srv/face_direction.hpp>
#include <nlohmann/json.hpp> // Assuming standard ROS 2 setup has nlohmann

// A helper function to parse simple JSON string array manually if nlohmann isn't present
std::vector<std::string> parseStringArray(const std::string& str) {
    std::vector<std::string> result;
    // Simple mock parser for ["item1", "item2"] since standard C++ lacks json
    std::string s = str;
    s.erase(std::remove(s.begin(), s.end(), '['), s.end());
    s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
    s.erase(std::remove(s.begin(), s.end(), '"'), s.end());
    s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
    
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) result.push_back(item);
    }
    return result;
}

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
            if (!client_->wait_for_service(std::chrono::milliseconds(0))) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Service %s not available, waiting...", service_name.c_str());
                return BT::NodeStatus::RUNNING;
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
            if (!client_->wait_for_service(std::chrono::milliseconds(0))) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Service /detect_center_kfs not available, waiting...");
                return BT::NodeStatus::RUNNING;
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

class UpdatePlannerMemory : public BT::SyncActionNode
{
public:
    UpdatePlannerMemory(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        publisher_ = node_->create_publisher<std_msgs::msg::String>("/planner/update_memory", 10);
    }

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<int>("block_id"),
            BT::InputPort<std::string>("kfs_type")
        };
    }

    BT::NodeStatus tick() override
    {
        int b_id = 0;
        std::string k_type = "";
        
        if (!getInput("block_id", b_id) || !getInput("kfs_type", k_type)) {
            RCLCPP_ERROR(node_->get_logger(), "UpdatePlannerMemory missing inputs");
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::String msg;
        msg.data = std::to_string(b_id) + ":" + k_type;
        publisher_->publish(msg);
        
        RCLCPP_INFO(node_->get_logger(), "Updated Planner Memory: Block %d = %s", b_id, k_type.c_str());
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class ExecutePickKFS : public BT::ActionNodeBase
{
public:
    ExecutePickKFS(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::ActionNodeBase(name, config), node_(node), request_sent_(false)
    {
        client_ = node_->create_client<robocon_interfaces::srv::PickKFS>("/pick_kfs");
    }

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<int>("current_block"),
            BT::InputPort<int>("target_block")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!request_sent_) {
            if (!client_->wait_for_service(std::chrono::milliseconds(0))) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Service /pick_kfs not available, waiting...");
                return BT::NodeStatus::RUNNING;
            }
            auto req = std::make_shared<robocon_interfaces::srv::PickKFS::Request>();
            
            int c_block = 0, t_block = 0;
            getInput("current_block", c_block);
            getInput("target_block", t_block);
            
            req->current_block = c_block;
            req->target_block = t_block;

            future_ = client_->async_send_request(req).future.share();
            request_sent_ = true;
            return BT::NodeStatus::RUNNING;
        }

        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            request_sent_ = false;
            try {
                auto result = future_.get();
                if (result->success) {
                    RCLCPP_INFO(node_->get_logger(), "PickKFS success: %s", result->message.c_str());
                    return BT::NodeStatus::SUCCESS;
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "PickKFS failed: %s", result->message.c_str());
                    return BT::NodeStatus::FAILURE;
                }
            } catch (...) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to call /pick_kfs service");
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
    rclcpp::Client<robocon_interfaces::srv::PickKFS>::SharedPtr client_;
    std::shared_future<robocon_interfaces::srv::PickKFS::Response::SharedPtr> future_;
    bool request_sent_;
};

class ExecuteFaceDirection : public BT::ActionNodeBase
{
public:
    ExecuteFaceDirection(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::ActionNodeBase(name, config), node_(node), request_sent_(false)
    {
        client_ = node_->create_client<robocon_interfaces::srv::FaceDirection>("/face_direction");
    }

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<int>("current_block_id"),
            BT::InputPort<std::string>("hl_params"),
            BT::OutputPort<int>("new_facing_block_id")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!request_sent_) {
            if (!client_->wait_for_service(std::chrono::milliseconds(0))) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Service /face_direction not available, waiting...");
                return BT::NodeStatus::RUNNING;
            }
            auto req = std::make_shared<robocon_interfaces::srv::FaceDirection::Request>();
            
            int c_block = 0;
            getInput("current_block_id", c_block);
            req->current_block_id = c_block;

            std::string params;
            getInput("hl_params", params);
            
            // Extract direction from {"direction": "left"}
            std::string dir = "front";
            try {
                auto j = nlohmann::json::parse(params);
                if (j.contains("direction")) {
                    dir = j["direction"].get<std::string>();
                }
            } catch (...) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to parse hl_params for direction");
            }
            req->direction = dir;

            future_ = client_->async_send_request(req).future.share();
            request_sent_ = true;
            return BT::NodeStatus::RUNNING;
        }

        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            request_sent_ = false;
            try {
                auto result = future_.get();
                if (result->success) {
                    RCLCPP_INFO(node_->get_logger(), "FaceDirection success: %s", result->message.c_str());
                    
                    // Parse target_block_id from params and set it to blackboard
                    std::string params;
                    getInput("hl_params", params);
                    try {
                        auto j = nlohmann::json::parse(params);
                        if (j.contains("target_block_id")) {
                            int target_id = j["target_block_id"].get<int>();
                            setOutput("new_facing_block_id", target_id);
                        }
                    } catch (...) {}
                    
                    return BT::NodeStatus::SUCCESS;
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "FaceDirection failed: %s", result->message.c_str());
                    return BT::NodeStatus::FAILURE;
                }
            } catch (...) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to call /face_direction service");
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
    rclcpp::Client<robocon_interfaces::srv::FaceDirection>::SharedPtr client_;
    std::shared_future<robocon_interfaces::srv::FaceDirection::Response::SharedPtr> future_;
    bool request_sent_;
};

class GetHighLevelPlan : public BT::ActionNodeBase
{
public:
    GetHighLevelPlan(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::ActionNodeBase(name, config), node_(node), request_sent_(false)
    {
        client_ = node_->create_client<robocon_interfaces::srv::GetHighLevelPlan>("/get_high_level_plan");
    }

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<int>("current_block_id"),
            BT::InputPort<int>("current_facing_block_id"),
            BT::InputPort<int>("current_kfs_count"),
            BT::InputPort<bool>("gripper_has_kfs"),
            BT::OutputPort<std::string>("sequence_out"),
            BT::OutputPort<std::string>("params_out")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!request_sent_) {
            if (!client_->wait_for_service(std::chrono::milliseconds(0))) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Service /get_high_level_plan not available, waiting...");
                return BT::NodeStatus::RUNNING;
            }
            auto req = std::make_shared<robocon_interfaces::srv::GetHighLevelPlan::Request>();
            
            int block_id = 4;
            getInput("current_block_id", block_id);
            req->current_block_id = block_id;
            
            int facing_id = 5;
            getInput("current_facing_block_id", facing_id);
            req->current_facing_block_id = facing_id;
            
            int count = 0;
            getInput("current_kfs_count", count);
            req->current_kfs_count = count;
            
            bool has_kfs = false;
            getInput("gripper_has_kfs", has_kfs);
            req->gripper_has_kfs = has_kfs;

            future_ = client_->async_send_request(req).future.share();
            request_sent_ = true;
            return BT::NodeStatus::RUNNING;
        }

        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            request_sent_ = false;
            try {
                auto result = future_.get();
                setOutput("sequence_out", result->sequence_name);
                setOutput("params_out", result->sequence_params_json);
                RCLCPP_INFO(node_->get_logger(), "Got High Level Plan: %s", result->sequence_name.c_str());
                return BT::NodeStatus::SUCCESS;
            } catch (...) {
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void halt() override { request_sent_ = false; }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<robocon_interfaces::srv::GetHighLevelPlan>::SharedPtr client_;
    std::shared_future<robocon_interfaces::srv::GetHighLevelPlan::Response::SharedPtr> future_;
    bool request_sent_;
};

class GetLowLevelPlan : public BT::ActionNodeBase
{
public:
    GetLowLevelPlan(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::ActionNodeBase(name, config), node_(node), request_sent_(false)
    {
        client_ = node_->create_client<robocon_interfaces::srv::GetLowLevelPlan>("/get_low_level_plan");
    }

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("sequence_in"),
            BT::InputPort<std::string>("params_in"),
            BT::OutputPort<std::string>("subseqs_out"),
            BT::OutputPort<std::string>("subseq_params_out")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!request_sent_) {
            if (!client_->wait_for_service(std::chrono::milliseconds(0))) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Service /get_low_level_plan not available, waiting...");
                return BT::NodeStatus::RUNNING;
            }
            auto req = std::make_shared<robocon_interfaces::srv::GetLowLevelPlan::Request>();
            
            std::string seq, params;
            getInput("sequence_in", seq);
            getInput("params_in", params);
            req->sequence_name = seq;
            req->sequence_params_json = params;

            future_ = client_->async_send_request(req).future.share();
            request_sent_ = true;
            return BT::NodeStatus::RUNNING;
        }

        if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            request_sent_ = false;
            try {
                auto result = future_.get();
                // Join the arrays into a comma separated string for the BT port
                std::string subseq_str = "";
                for(const auto& s : result->subsequences) subseq_str += s + ",";
                
                std::string params_str = "";
                for(const auto& p : result->subsequence_params_json) params_str += p + "||";

                setOutput("subseqs_out", subseq_str);
                setOutput("subseq_params_out", params_str);
                return BT::NodeStatus::SUCCESS;
            } catch (...) {
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void halt() override { request_sent_ = false; }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<robocon_interfaces::srv::GetLowLevelPlan>::SharedPtr client_;
    std::shared_future<robocon_interfaces::srv::GetLowLevelPlan::Response::SharedPtr> future_;
    bool request_sent_;
};

class ExecuteLowLevelPlan : public BT::StatefulActionNode
{
public:
    ExecuteLowLevelPlan(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node), current_idx_(0), waiting_for_hardware_(false)
    {
        // Setup generic trigger client for hardware
        hardware_client_ = node_->create_client<std_srvs::srv::Trigger>("/hardware_trigger");
    }

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("subseqs_in"),
            BT::InputPort<std::string>("subseq_params_in")
        };
    }

    BT::NodeStatus onStart() override
    {
        std::string subseqs_str, params_str;
        if (!getInput("subseqs_in", subseqs_str) || !getInput("subseq_params_in", params_str)) {
            return BT::NodeStatus::FAILURE;
        }
        
        // Split by comma
        subsequences_.clear();
        std::stringstream ss(subseqs_str);
        std::string item;
        while (std::getline(ss, item, ',')) {
            if (!item.empty()) subsequences_.push_back(item);
        }
        
        current_idx_ = 0;
        waiting_for_hardware_ = false;
        
        RCLCPP_INFO(node_->get_logger(), "Executing Low Level Plan with %lu steps.", subsequences_.size());
        if (subsequences_.empty()) return BT::NodeStatus::SUCCESS;
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (current_idx_ >= subsequences_.size()) {
            return BT::NodeStatus::SUCCESS;
        }

        if (!waiting_for_hardware_) {
            // Send request to hardware controller
            std::string current_cmd = subsequences_[current_idx_];
            RCLCPP_INFO(node_->get_logger(), "-> Executing step %lu: %s", current_idx_ + 1, current_cmd.c_str());
            
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            future_ = hardware_client_->async_send_request(req).future.share();
            waiting_for_hardware_ = true;
            return BT::NodeStatus::RUNNING;
        } else {
            // Wait for hardware to finish the command
            if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                waiting_for_hardware_ = false;
                current_idx_++; // Move to next command
                
                if (current_idx_ >= subsequences_.size()) {
                    RCLCPP_INFO(node_->get_logger(), "All hardware steps completed!");
                    return BT::NodeStatus::SUCCESS;
                }
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override { current_idx_ = 0; waiting_for_hardware_ = false; }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hardware_client_;
    std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> future_;
    std::vector<std::string> subsequences_;
    size_t current_idx_;
    bool waiting_for_hardware_;
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

    factory.registerBuilder<GetHighLevelPlan>("GetHighLevelPlan", 
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<GetHighLevelPlan>(name, config, node);
        });

    factory.registerBuilder<GetLowLevelPlan>("GetLowLevelPlan", 
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<GetLowLevelPlan>(name, config, node);
        });

    factory.registerBuilder<ExecuteLowLevelPlan>("ExecuteLowLevelPlan", 
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<ExecuteLowLevelPlan>(name, config, node);
        });

    factory.registerBuilder<UpdatePlannerMemory>("UpdatePlannerMemory", 
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<UpdatePlannerMemory>(name, config, node);
        });

    factory.registerBuilder<ExecutePickKFS>("ExecutePickKFS", 
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<ExecutePickKFS>(name, config, node);
        });

    factory.registerBuilder<ExecuteFaceDirection>("ExecuteFaceDirection", 
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<ExecuteFaceDirection>(name, config, node);
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
