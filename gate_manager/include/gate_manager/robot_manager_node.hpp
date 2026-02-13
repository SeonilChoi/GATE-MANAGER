#ifndef GATE_MANAGER_ROBOT_MANAGER_NODE_HPP_
#define GATE_MANAGER_ROBOT_MANAGER_NODE_HPP_

#include <string>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <gate_msgs/msg/robot_state_multi_array.hpp>

#include "micros_common_type/micros_common_type.hpp"

namespace micros {

class RobotManagerNode : public rclcpp::Node {
public:
    using MotorStateMultiArray = gate_msgs::msg::MotorStateMultiArray;

    explicit RobotManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    virtual ~RobotManagerNode() override;

private:
    void timer_callback();

    void motor_state_callback(const MotorStateMultiArray::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<MotorStateMultiArray>::SharedPtr motor_state_subscriber_;
    
    rclcpp::Publisher<MotorStateMultiArray>::SharedPtr motor_command_publisher_;

    std::string config_file_;
    
    std::unique_ptr<RobotManager> robot_manager_;

    uint8_t size_{0};
    
    motor_state_t states_[MAX_CONTROLLER_SIZE]{};
};

} // namespace micros
#endif // GATE_MANAGER_ROBOT_MANAGER_NODE_HPP_