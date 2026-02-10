#ifndef GATE_MANAGER_MOTOR_MANAGER_NODE_HPP_
#define GATE_MANAGER_MOTOR_MANAGER_NODE_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <gate_msgs/msg/motor_state_multi_array.hpp>

#include "micros_common_type/micros_common_type.hpp"
#include "motor_manager/motor_manager.hpp"

namespace micros {

class MotorManagerNode : public rclcpp::Node {

public:
    using MotorStateMultiArray = gate_msgs::msg::MotorStateMultiArray;

    explicit MotorManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~MotorManagerNode() = default;

private:
    void motor_command_callback(const MotorStateMultiArray::SharedPtr msg);

    void setup();

    void loop();

    rclcpp::Subscription<MotorStateMultiArray>::SharedPtr motor_command_subscriber_;
    rclcpp::Publisher<MotorStateMultiArray>::SharedPtr motor_state_publisher_;

    std::string config_file_;
    std::unique_ptr<MotorManager> motor_manager_;

    motor_state_gate_t motor_state_gate_;
};

} // namespace micros
#endif // #ifndef GATE_MANAGER_MOTOR_MANAGER_NODE_HPP_