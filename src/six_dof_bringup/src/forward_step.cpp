#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using namespace std::placeholders;

#define M_PI 3.14159265358979323846

double deg_to_rad(double degree) { return degree * M_PI / 180; }

class Position_controller : public rclcpp::Node {
public:
  Position_controller() : Node("pos_control") {

    // callback group
    imu_feedback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    joint_control_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 1);
    timer_ = this->create_wall_timer(
        225ms, std::bind(&Position_controller::timer_callback, this),
        joint_control_group);

    auto sub_opts = rclcpp::SubscriptionOptions();
    sub_opts.callback_group = imu_feedback_group;

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 1, std::bind(&Position_controller::imu_callback, this, _1),
        sub_opts);
  }

private:
  // ROS2
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::CallbackGroup::SharedPtr imu_feedback_group, joint_control_group;

  double right_thigh_ = -0.12; //-0.1
  double left_thigh_ = -right_thigh_;
  double right_knee_ = 2 * left_thigh_; // 0.2
  double left_knee_ = 2 * left_thigh_;
  double right_foot_ = -0.25;
  double left_foot_ = -0.25;
  double right_thigh_lateral_ = deg_to_rad(1);
  double left_thigh_lateral_ = deg_to_rad(1);

  uint32_t delivery_time = 110'000'000;

  // for lateral - left shift its left=0.1 and right =-0.1
  // for right invert= left= -0.1 and right =0.1

  void timer_callback() {
    trajectory_msgs::msg::JointTrajectory move_msg =
        trajectory_msgs::msg::JointTrajectory(); // Directly instantiate the
                                                 // message object

    // msg.header.stamp = this->get_clock()->now();
    move_msg.joint_names = {"body_left_thigh",  "body_left_thigh_lateral",
                            "right_body_thigh", "right_body_thigh_lateral",
                            "right_thigh_shin", "left_thigh_shin",
                            "right_shin_foot",  "left_shin_foot"};
    move_msg.header.frame_id = "";

    trajectory_msgs::msg::JointTrajectoryPoint point1, point2, point3, point4;

    // left lift and righ shift(-left ,+ve right. both input values are
    // positive)
    point1.positions = {
        -left_thigh_, -left_thigh_lateral_, 0.0, right_thigh_lateral_,
        0.0,         left_knee_,           0.0, left_foot_};
    // Using nanoseconds for specifying the time_from_start

    point1.time_from_start = rclcpp::Duration(0, delivery_time);

    // point2.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // point2.time_from_start = rclcpp::Duration(0, 2 * delivery_time);

    // right lift and left shift
    point3.positions = {0.0,
                        left_thigh_lateral_,
                        right_thigh_ - 0.1,
                        -right_thigh_lateral_,
                        right_knee_,
                        0.0,
                        right_foot_,
                        0.0};
    point3.time_from_start = rclcpp::Duration(0, 2 * delivery_time);

    point2.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point2.time_from_start = rclcpp::Duration(0, 3 * delivery_time);

    // point4.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // point4.time_from_start = rclcpp::Duration(0, 4 * delivery_time);
    move_msg.points.push_back(point1);
    move_msg.points.push_back(point3);
    move_msg.points.push_back(point2);
    // Publish the message
    joint_pub_->publish(move_msg); // Note: No need to dereference msg here
  }

  void imu_callback(sensor_msgs::msg::Imu imu_msg) {
    auto orientation = imu_msg.orientation;
    RCLCPP_INFO(this->get_logger(),
                "orientation quaterion[w,x,y,z] is [%f,%f,%f,%f]",
                orientation.w, orientation.x, orientation.y, orientation.z);
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