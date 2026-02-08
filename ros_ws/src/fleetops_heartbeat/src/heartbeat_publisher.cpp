#include <chrono> // this header is needed for std::chrono::milliseconds. This is required for the sleep_for function to work properly.
#include <memory> // this header is needed for std::make_shared. This is required for creating shared pointers to the HeartbeatPublisher class.
#include <string> // this header is needed for std::string. This is required for using string data types in the code.

#include <rclcpp/rclcpp.hpp> // this header is needed for ROS 2 C++ client library. This is required for creating ROS 2 nodes and publishers.
#include "fleetops_msgs/msg/heartbeat.hpp" // this header is needed for the Heartbeat message type. This is required for publishing heartbeat messages.

using namespace std::chrono_literals; // this line allows us to use the 'ms' suffix for milliseconds when specifying durations.

class HeartbeatPublisher : public rclcpp::Node //we use rclcpp::Node as the base class for our HeartbeatPublisher class, which allows us to create a ROS 2 node that can publish messages.
{
public: // this is the constructor for the HeartbeatPublisher class. It initializes the node, creates a publisher for the Heartbeat message type, and sets up a timer to call the timer_callback function at regular intervals.
  HeartbeatPublisher()  // this line initializes the HeartbeatPublisher class and calls the constructor of the base class rclcpp::Node with the name "heartbeat_publisher". It also initializes the sequence number (seq_) to 0. rclcpp means ROS Client Library for C++. The constructor is responsible for setting up the publisher and timer for the heartbeat messages.
  : Node("heartbeat_publisher"), seq_(0) 
  {
    robot_id_ = this->declare_parameter<std::string>("robot_id","robot_1"); // this line declares a parameter named "robot_id" with a default value of "robot_1". This allows the user to specify the robot ID when launching the node, and if not specified, it will use "robot_1" as the default.
    publish_hz_ = this->declare_parameter<double>("publish_hz",5.0); // this line declares a parameter named "publish_hz" with a default value of 5.0. This allows the user to specify the publishing frequency in hertz when launching the node, and if not specified, it will use 5.0 Hz as the default.

    rclcpp::QoS qos(rclcpp::KeepLast(10)); // this line creates a Quality of Service (QoS) object with a history policy of "KeepLast" and a depth of 10. This means that the publisher will keep the last 10 messages in its history, which can be useful for ensuring that subscribers receive the most recent messages even if they are not able to keep up with the publishing rate.
    qos.best_effort(); // this line sets the QoS reliability policy to "Best Effort". This means that the publisher will not guarantee that all messages are delivered to subscribers, and some messages may be lost if the network is congested or if subscribers are not able to keep up with the publishing rate. This is often used for non-critical data where occasional message loss is acceptable.

    publisher_ = this->create_publisher<fleetops_msgs::msg::Heartbeat>("/fleet/heartbeat", qos); // this line creates a publisher for the Heartbeat message type on the "/fleet/heartbeat" topic using the previously defined QoS settings. This allows the node to publish heartbeat messages to any subscribers that are listening on this topic.

    auto period = std::chrono::duration<double>(1.0 / publish_hz_); // this line calculates the period between each heartbeat message based on the publishing frequency (publish_hz_). It creates a duration object that represents the time interval between messages, which is used to set up the timer for periodic callbacks.
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period), 
        std::bind(&HeartbeatPublisher::on_timer, this)); // this line creates a wall timer that calls the on_timer function at regular intervals defined by the period calculated in the previous line. The timer will trigger the on_timer function, which is responsible for publishing the heartbeat messages at the specified frequency.
  }

  private:
    void on_timer() // this is the callback function that gets called every time the timer triggers. It creates a Heartbeat message, populates it with the current sequence number and robot ID, and publishes it to the "/fleet/heartbeat" topic.
    {
        fleetops_msgs::msg::Heartbeat msg; // this line creates an instance of the Heartbeat message type. This message will be populated with the necessary data before being published.
        msg.robot_id = robot_id_; // this line sets the robot_id field of the Heartbeat message to the value of the robot_id_ member variable, which was declared as a parameter in the constructor. This allows the heartbeat message to include the identifier of the robot that is sending the heartbeat.
        msg.stamp = this->now(); // this line sets the stamp field of the Heartbeat message to the current time using the now() function provided by the rclcpp::Node class. This timestamp indicates when the heartbeat message was created, which can be useful for subscribers to determine the freshness of the data.
        msg.seq = seq_++; // this line sets the seq field of the Heartbeat message to the current value of the seq_ member variable and then increments seq_ by 1. This sequence number can be used by subscribers to track the order of received heartbeat messages and detect any missed messages

        publisher_->publish(msg); // this line publishes the populated Heartbeat message to the "/fleet/heartbeat" topic using the publisher_ member variable. This allows any subscribers that are listening on this topic to receive the heartbeat message.
    }

    std::string robot_id_; // this member variable stores the robot ID, which is declared as a parameter in the constructor. It is used to populate the robot_id field of the Heartbeat message.
    double publish_hz_; // this member variable stores the publishing frequency in hertz, which is declared as a parameter in the constructor. It is used to calculate the period for the timer that triggers the on_timer function.
    uint32_t seq_; // this member variable stores the sequence number for the heartbeat messages. It is initialized to 0 in the constructor and incremented each time a heartbeat message is published.
    rclcpp::Publisher<fleetops_msgs::msg::Heartbeat>::SharedPtr publisher_; // this member variable is a shared pointer to the publisher for the Heartbeat message type. It is created in the constructor and used to publish heartbeat messages in the on_timer function.
    rclcpp::TimerBase::SharedPtr timer_; // this member variable is a shared pointer to the timer that triggers the on_timer function at regular intervals. It is created in the constructor.
};

int main(int argc, char * argv[]) // this is the main function that initializes the ROS 2 system, creates an instance of the HeartbeatPublisher class, and starts the ROS 2 event loop to process callbacks.
{
  rclcpp::init(argc, argv); // this line initializes the ROS 2 system with the command-line arguments. This is necessary before creating any nodes or publishers.
  rclcpp::spin(std::make_shared<HeartbeatPublisher>()); // this line creates a shared pointer to an instance of the HeartbeatPublisher class and starts spinning it. The spin function will keep the node alive and allow it to process callbacks, such as the timer callback that publishes heartbeat messages.
  rclcpp::shutdown(); // this line shuts down the ROS 2 system when the node is no longer needed. This is typically called after spinning is complete or when the program is exiting.
  return 0; // this line returns 0 to indicate that the program has completed successfully.
}