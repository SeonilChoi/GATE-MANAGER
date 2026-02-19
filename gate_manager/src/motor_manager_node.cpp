#include <cerrno>
#include <cstring>
#include <csignal>

#include <time.h>
#include <sched.h>
#include <unistd.h>
#include <sys/mman.h>

#include "gate_manager/motor_manager_node.hpp"

namespace micros {

inline constexpr std::size_t MAX_SAFE_STACK = 8 * 1024;

static void stack_prefault()
{
    unsigned char dummy[MAX_SAFE_STACK]{};
    std::memset(dummy, 0, MAX_SAFE_STACK);
}

MotorManagerNode::MotorManagerNode(const rclcpp::NodeOptions& options)
: Node("motor_manager_node", options) {
    motor_command_subscriber_ = this->create_subscription<MotorStateMultiArray>(
        "/motor_command", rclcpp::QoS(1).best_effort(),
        [this](const MotorStateMultiArray::SharedPtr msg) {
            motor_command_callback(msg);
        }
    );

    motor_state_publisher_ = this->create_publisher<MotorStateMultiArray>(
        "/motor_state", rclcpp::QoS(10).best_effort()
    );
    
    config_file_ = this->declare_parameter<std::string>("config_file", "");
    
    motor_manager_ = std::make_unique<MotorManager>(config_file_);

    is_running_.store(true, std::memory_order_relaxed);
    rt_thread_ = std::thread([this]() { this->loop(); });
    RCLCPP_INFO(this->get_logger(), "RT thread started");
}

~MotorManagerNode() override 
{
    is_running_.store(false, std::memory_order_relaxed);
    if (rt_thread_.joinable()) {
        rt_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "RT thread joined");
}

void MotorManagerNode::motor_command_callback(const MotorStateMultiArray::SharedPtr msg)
{
    motor_state_t data[MAX_CONTROLLER_SIZE]{};
    convert_from_ros_message<MotorStateMultiArray>(*msg, data);
    motor_state_gate_.write(data, static_cast<uint8_t>(msg->data.size()));
}

void MotorManagerNode::setup()
{
    struct sched_param param{};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        throw std::runtime_error("sched_setscheduler failed: " + std::string(std::strerror(errno)));
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        throw std::runtime_error("mlockall failed: " + std::string(std::strerror(errno)));
    }

    stack_prefault();
}

void MotorManagerNode::loop()
{
    try {
        setup();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "RT setup failed: %s", e.what());
        return;
    }

    motor_manager_->start();

    struct timespec wakeup_time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1;
    wakeup_time.tv_nsec = 0;

    uint64_t last_count = 0;
    
    const uint32_t period = motor_manager_->period();
    const uint8_t size = motor_manager_->number_of_controllers();

    while (rclcpp::ok() && is_running_.load(std::memory_order_relaxed)) {
        const int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, nullptr);
        if (ret == EINTR) {
            continue;
        }
        if (ret) {
            RCLCPP_ERROR(this->get_logger(), "clock_nanosleep failed: %s", std::strerror(errno));
            break;
        }

        motor_state_t states[MAX_CONTROLLER_SIZE]{};
        motor_state_t cmds[MAX_CONTROLLER_SIZE]{};

        (void)motor_state_gate_.read(cmds, size, last_count);

        const bool stop = !rclcpp::ok();
        if (motor_manager_->update(stop, states, cmds)) {
            break;
        }

        MotorStateMultiArray::SharedPtr msg = std::make_shared<MotorStateMultiArray>();
        convert_to_ros_message<MotorStateMultiArray>(states, size, *msg);
        motor_state_publisher_->publish(*msg);
    
        wakeup_time.tv_nsec += period;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_sec++;
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
        }
    }

    motor_manager_->stop();
}

} // namespace micros

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<micros::MotorManagerNode>();
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}   