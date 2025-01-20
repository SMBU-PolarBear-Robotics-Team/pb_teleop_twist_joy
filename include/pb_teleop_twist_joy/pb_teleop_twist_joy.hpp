// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PB_TELEOP_TWIST_JOY__PB_TELEOP_TWIST_JOY_HPP_
#define PB_TELEOP_TWIST_JOY__PB_TELEOP_TWIST_JOY_HPP_

#include <map>
#include <memory>
#include <string>

#include "example_interfaces/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb_teleop_twist_joy
{

class TeleopTwistJoyNode : public rclcpp::Node
{
public:
  explicit TeleopTwistJoyNode(const rclcpp::NodeOptions & options);

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map);
  void fillCmdVelMsg(
    const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map,
    geometry_msgs::msg::Twist * cmd_vel_msg);
  void fillJointStateMsg(
    const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map,
    sensor_msgs::msg::JointState * joint_state_msg);
  void fillShootMsg(
    const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map,
    example_interfaces::msg::UInt8 * shoot_msg);
  void sendGoalPoseAction(
    const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map);
  void sendZeroCommand();
  double getVal(
    const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t> & axis_map,
    const std::map<std::string, double> & scale_map, const std::string & fieldname);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool publish_stamped_twist_;
  std::string robot_base_frame_;
  std::string control_mode_;
  bool require_enable_button_;
  int64_t enable_button_;
  int64_t enable_turbo_button_;
  bool inverted_reverse_;

  std::map<std::string, int64_t> axis_chassis_map_;
  std::map<std::string, std::map<std::string, double>> scale_chassis_map_;
  std::map<std::string, int64_t> axis_gimbal_map_;
  std::map<std::string, std::map<std::string, double>> scale_gimbal_map_;

  bool sent_disable_msg_;
  double dt_;
};

}  // namespace pb_teleop_twist_joy

#endif  // PB_TELEOP_TWIST_JOY__PB_TELEOP_TWIST_JOY_HPP_
