#include "gate_manager/robot_manager_node.hpp"

using namespace std::chrono_literals;

namespace micros {

RobotManagerNode::RobotManagerNode(const rclcpp::NodeOptions& options)
: Node("robot_manager_node", options) {
    timer_ = this->create_wall_timer(10ms, [this]() { timer_callback(); });

    motor_state_subscriber_ = this->create_subscription<MotorStateMultiArray>(
        "/motor_state", rclcpp::QoS(10).best_effort(),
        [this](const MotorStateMultiArray::SharedPtr msg) {
            motor_state_callback(msg);
        }
    );

    motor_command_publisher_ = this->create_publisher<MotorStateMultiArray>(
        "/motor_command", rclcpp::QoS(1).best_effort()
    );

    config_file_ = this->declare_parameter<std::string>("config_file", "");

    robot_manager_ = std::make_unique<RobotManager>(config_file_);
}

void RobotManagerNode::timer_callback()
{
    uint8_t size = 0;
    motor_state_t cmds[MAX_CONTROLLER_SIZE]{};
    robot_manager_->control(cmds, size);

    MotorStateMultiArray::SharedPtr msg = std::make_shared<MotorStateMultiArray>();
    convert_to_ros_message<MotorStateMultiArray>(cmds, size, *msg);
    motor_command_publisher_->publish(*msg);
}

void RobotManagerNode::motor_state_callback(const MotorStateMultiArray::SharedPtr msg)
{
    uint8_t size = static_cast<uint8_t>(msg->data.size());
    motor_state_t states[MAX_CONTROLLER_SIZE]{};
    convert_from_ros_message<MotorStateMultiArray>(*msg, size, states);
    robot_manager_->update(states, size);
}

} // namespace micros

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<micros::RobotManagerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}