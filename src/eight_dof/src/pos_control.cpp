#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

#define M_PI 3.14159265358979323846

double deg_to_rad(double degree) { return degree * M_PI / 180; }

class Position_controller : public rclcpp::Node {
public:
  Position_controller() : Node("pos_control") {
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    // qos.reliable();
    // qos.durability_volatile();
    // qos.deadline(std::chrono::milliseconds(400));
    // qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);
    timer_ = this->create_wall_timer(
        445ms, std::bind(&Position_controller::timer_callback, this));
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double right_thigh_ = 0.3; 
  double left_thigh_ = -right_thigh_;
  double right_knee_ = -0.35; 
  double left_knee_ = - right_knee_;
  double right_foot_ = -0.2; 
  double left_foot_ = -0.2;
  double right_thigh_lateral_ = deg_to_rad(0);
  double left_thigh_lateral_ = deg_to_rad(0);
  double turn_thigh=0.025;
  double turn_knee=turn_thigh*2;

  uint32_t delivery_time = 110'000'000;

  // for lateral - left shift its left=0.1 and right =-0.1
  // for right invert= left= -0.1 and right =0.1

  void timer_callback() {
    trajectory_msgs::msg::JointTrajectory forward_msg =
        trajectory_msgs::msg::JointTrajectory(); // Directly instantiate the
                                                 // message object

    // msg.header.stamp = this->get_clock()->now();
    forward_msg.joint_names = {"hip_left",  "hip_left_lateral",
                               "hip_right", "hip_right_lateral",
                               "right_knee", "left_knee",
                               "right_ankle",  "left_ankle"};
    forward_msg.header.frame_id = "";

    trajectory_msgs::msg::JointTrajectoryPoint point1, point2, point3, point4;

    // left lift and righ shift(-left ,+ve right. both input values are
    // positive)
    point1.positions = {
        left_thigh_ , -left_thigh_lateral_, 0.0, right_thigh_lateral_,
        0.0,         left_knee_,           0.0, left_foot_};
    // Using nanoseconds for specifying the time_from_start

    point1.time_from_start = rclcpp::Duration(0, delivery_time);

    point2.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point2.time_from_start = rclcpp::Duration(0, 2 * delivery_time);

    // right lift and left shift
    point3.positions = {0.0,          left_thigh_lateral_,
                        right_thigh_  , -right_thigh_lateral_,
                        right_knee_ ,  0.0,
                        right_foot_,  0.0};
    point3.time_from_start = rclcpp::Duration(0, 3 * delivery_time);

    point4.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point4.time_from_start = rclcpp::Duration(0, 4 * delivery_time);

    // std::vector<double> v1,v2,v3,v4;

    // Add points to the message
    forward_msg.points.push_back(point1);
    forward_msg.points.push_back(point2);
    forward_msg.points.push_back(point3);
    forward_msg.points.push_back(point4);

    // Publish the message
    pub_->publish(forward_msg); // Note: No need to dereference msg here
  }

};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto pos_control_node = std::make_shared<Position_controller>();
  auto mt_exe = rclcpp::executors::MultiThreadedExecutor();
  mt_exe.add_node(pos_control_node);
  mt_exe.spin();
  rclcpp::shutdown();

  return 0;
}