#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SCD30Publisher : public rclcpp::Node {
public:
  SCD30Publisher() : Node("scd30_sensor_1") {
    publisher_ =
        this->create_publisher<std_msgs::msg::Float32>("scd30_sensor_1", 10);
    timer_ = this->create_wall_timer(
        2s, std::bind(&SCD30Publisher::publish_o2, this));
  }

private:
  void publish_o2() {
    auto message = std_msgs::msg::Float32();
    message.data = 400 + static_cast<float>(rand()) /
                             (static_cast<float>(RAND_MAX / (700 - 400)));
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SCD30Publisher>());
  rclcpp::shutdown();
  return 0;
}