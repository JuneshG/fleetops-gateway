#include <chrono>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "fleetops_msgs/msg/heartbeat.hpp"
#include "fleetops_msgs/msg/fault.hpp"
#include "fleetops_msgs/msg/robot_health.hpp"
#include "fleetops_health/timeout.hpp"

using namespace std::chrono_literals;

struct RobotState
{
  rclcpp::Time last_seen;
  bool alive{true};
};

class HealthSupervisor : public rclcpp::Node
{
public:
  HealthSupervisor()
  : Node("health_supervisor")
  {
    timeout_sec_ = this->declare_parameter<double>("timeout_sec", 2.0);

    rclcpp::QoS hb_qos(rclcpp::KeepLast(50));
    hb_qos.best_effort();
    heartbeat_sub_ = this->create_subscription<fleetops_msgs::msg::Heartbeat>(
      "/fleet/heartbeat",
      hb_qos,
      std::bind(&HealthSupervisor::on_heartbeat, this, std::placeholders::_1));

    rclcpp::QoS reliable_qos(rclcpp::KeepLast(10));
    reliable_qos.reliable();
    faults_pub_ = this->create_publisher<fleetops_msgs::msg::Fault>("/fleet/faults", reliable_qos);
    health_pub_ =
      this->create_publisher<fleetops_msgs::msg::RobotHealth>("/fleet/health", reliable_qos);

    timer_ = this->create_wall_timer(100ms, std::bind(&HealthSupervisor::on_timer, this));
  }

private:
  void on_heartbeat(const fleetops_msgs::msg::Heartbeat::SharedPtr msg)
  {
    auto & state = robots_[msg->robot_id];
    const bool was_alive = state.alive;

    state.last_seen = msg->stamp;
    state.alive = true;

    if (!was_alive) {
      publish_fault(msg->robot_id, "HEARTBEAT_RECOVERED", "Heartbeat recovered", 0);
    }
  }

  void on_timer()
  {
    const auto now = this->now();

    for (auto & kv : robots_) {
      const std::string & robot_id = kv.first;
      RobotState & state = kv.second;

      const double age = (now - state.last_seen).seconds();
      const bool timed_out = is_timed_out(age, timeout_sec_);

      if (state.alive && timed_out) {
        state.alive = false;
        publish_fault(robot_id, "HEARTBEAT_LOST", "No heartbeat within timeout", 2);
      }

      publish_health(robot_id, state, age);
    }
  }

  void publish_fault(
    const std::string & robot_id,
    const std::string & code,
    const std::string & message,
    uint8_t severity)
  {
    fleetops_msgs::msg::Fault msg;
    msg.robot_id = robot_id;
    msg.stamp = this->now();
    msg.code = code;
    msg.message = message;
    msg.severity = severity;
    faults_pub_->publish(msg);
  }

  void publish_health(const std::string & robot_id, const RobotState & state, double age)
  {
    fleetops_msgs::msg::RobotHealth msg;
    msg.robot_id = robot_id;
    msg.stamp = this->now();
    msg.alive = state.alive;
    msg.last_seen_age_sec = static_cast<float>(age);
    msg.status = state.alive ? "OK" : "STALE";
    health_pub_->publish(msg);
  }

  double timeout_sec_;
  std::unordered_map<std::string, RobotState> robots_;

  rclcpp::Subscription<fleetops_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_;
  rclcpp::Publisher<fleetops_msgs::msg::Fault>::SharedPtr faults_pub_;
  rclcpp::Publisher<fleetops_msgs::msg::RobotHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthSupervisor>());
  rclcpp::shutdown();
  return 0;
}
