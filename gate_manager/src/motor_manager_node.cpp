#include "gate_manager/motor_manager_node.hpp"

using namespace std::chrono_literals;

namespace micros {

MotorManagerNode::MotorManagerNode(const rclcpp::NodeOptions& options)
: Node("motor_manager_node", options) {
    motor_command_subscriber_ = this->create_subscription<MotorStateMultiArray>(
        "/motor_command", 1,
        [this](const MotorStateMultiArray::SharedPtr msg) {
            motor_command_callback(msg);
        }
    );

    motor_state_publisher_ = this->create_publisher<MotorStateMultiArray>(
        "/motor_state", 10
    );
}

void MotorManagerNode::motor_command_callback(const MotorStateMultiArray::SharedPtr msg)
{
    motor_state_t data[MAX_CONTROLLER_SIZE]{};
    convert_from_ros_message<MotorStateMultiArray>(*msg, data);
    motor_state_gate_.write(data, static_cast<uint8_t>(msg->data.size()));
}

} // namespace micros

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<micros::MotorManagerNode>());
    rclcpp::shutdown();
    return 0;
}   