#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "fleetops_msgs/msg/heartbeat.hpp"

using namespace std::chrono_literals;

class HeartbeatPublisher : public rclcpp::Node
{
public:
  HeartbeatPublisher()
  : Node("heartbeat_publisher"), seq_(0)
  {
    robot_id_ = this->declare_parameter<std::string>("robot_id", "robot_1");
    publish_hz_ = this->declare_parameter<double>("publish_hz", 5.0);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    publisher_ =
      this->create_publisher<fleetops_msgs::msg::Heartbeat>("/fleet/heartbeat", qos);

    auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&HeartbeatPublisher::on_timer, this));
  }

private:
  void on_timer()
  {
    fleetops_msgs::msg::Heartbeat msg;
    msg.robot_id = robot_id_;
    msg.stamp = this->now();
    msg.seq = seq_++;

    publisher_->publish(msg);
  }

  std::string robot_id_;
  double publish_hz_;
  uint32_t seq_;
  rclcpp::Publisher<fleetops_msgs::msg::Heartbeat>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartbeatPublisher>());
  rclcpp::shutdown();
  return 0;
}
