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

}

void RobotManagerNode::motor_state_callback(const MotorStateMultiArray::SharedPtr msg)
{
    size_ = static_cast<uint8_t>(msg->data.size());
    for (uint8_t i = 0; i < size_; ++i) {
        states_[i].id = msg->data[i].id;
        states_[i].position = msg->data[i].position;
        states_[i].velocity = msg->data[i].velocity;
        states_[i].torque = msg->data[i].torque;
    }
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