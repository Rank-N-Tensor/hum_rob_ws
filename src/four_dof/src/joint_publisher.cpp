#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class Joint_State_Pub : public rclcpp::Node {

public:
  Joint_State_Pub(std::string topic_name) : Node("joint_state_pub_node") {

    publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(topic_name, 10);

    timer_ = this->create_wall_timer(
        33ms, std::bind(&Joint_State_Pub::joint_state_callback, this));

    start_time_ = this->now();
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  void joint_state_callback() {

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();

    // joint names
    msg.name = {"body_to_left_thigh", "body_to_right_thigh",
                "left_thigh_to_knee", "right_thigh_to_knee",
                "left_knee_to_foot",  "right_knee_to_foot"};

    // Sine wave parameters
    double amplitude = 0.266; // Adjusted amplitude to match joint limits
    double frequency = 1.0;   // Frequency in Hz

    // Calculate elapsed time in seconds
    double elapsed_time = (this->now() - start_time_).seconds();

    // Calculate sinusoidal positions for a basic walking pattern
    msg.position.clear();
    for (size_t i = 0; i < msg.name.size(); ++i) {
      double phase = 2.0 * M_PI * frequency * elapsed_time;
      double position =
          amplitude * sin(phase + (i % 2) * M_PI); // Alternate legs
      msg.position.push_back(position);
    }

    msg.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    msg.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "publishing joint position");
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  Joint_State_Pub::SharedPtr joint_state_pub_node =
      std::make_shared<Joint_State_Pub>("/joint_states");

  rclcpp::spin(joint_state_pub_node);

  rclcpp::shutdown();

  return 0;
}