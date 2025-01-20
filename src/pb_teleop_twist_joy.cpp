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

#include "pb_teleop_twist_joy/pb_teleop_twist_joy.hpp"

#include <cinttypes>

namespace pb_teleop_twist_joy
{

TeleopTwistJoyNode::TeleopTwistJoyNode(const rclcpp::NodeOptions & options)
: Node("teleop_twist_joy_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting Teleop Twist Joy");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  this->declare_parameter<bool>("publish_stamped_twist", false);
  this->declare_parameter<std::string>("robot_base_frame", "base_link");
  this->declare_parameter<bool>("require_enable_button", true);
  this->declare_parameter<int64_t>("enable_button", 5);
  this->declare_parameter<int64_t>("enable_turbo_button", -1);
  this->declare_parameter<bool>("inverted_reverse", false);
  this->declare_parameter<std::string>("control_mode", "manual_control");

  this->declare_parameters<int64_t>("axis_chassis", {{"x", 5L}, {"y", -1L}, {"yaw", -1L}});
  this->declare_parameters<int64_t>(
    "axis_gimbal", {{"yaw", 3L}, {"pitch", 4L}, {"roll", -1L}, {"shoot", 5L}});
  this->declare_parameters<double>("scale_chassis", {{"x", 0.5}, {"y", 0.0}, {"yaw", 0.0}});
  this->declare_parameters<double>("scale_chassis_turbo", {{"x", 1.0}, {"y", 0.0}, {"yaw", 0.0}});
  this->declare_parameters<double>(
    "scale_gimbal", {{"yaw", 0.5}, {"pitch", 0.0}, {"roll", 0.0}, {"shoot", 1.0}});
  this->declare_parameters<double>(
    "scale_gimbal_turbo", {{"yaw", 1.0}, {"pitch", 0.0}, {"roll", 0.0}, {"shoot", 1.0}});

  this->get_parameter("publish_stamped_twist", publish_stamped_twist_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("require_enable_button", require_enable_button_);
  this->get_parameter("enable_button", enable_button_);
  this->get_parameter("enable_turbo_button", enable_turbo_button_);
  this->get_parameter("inverted_reverse", inverted_reverse_);
  this->get_parameter("control_mode", control_mode_);
  this->get_parameters("axis_chassis", axis_chassis_map_);
  this->get_parameters("axis_gimbal", axis_gimbal_map_);
  this->get_parameters("scale_chassis", scale_chassis_map_["normal"]);
  this->get_parameters("scale_chassis_turbo", scale_chassis_map_["turbo"]);
  this->get_parameters("scale_gimbal", scale_gimbal_map_["normal"]);
  this->get_parameters("scale_gimbal_turbo", scale_gimbal_map_["turbo"]);

  if (control_mode_ == "auto_control") {
    nav_to_pose_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
  }

  if (publish_stamped_twist_) {
    cmd_vel_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
  } else {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd_gimbal_joint", 10);
  shoot_pub_ = this->create_publisher<example_interfaces::msg::UInt8>("cmd_shoot", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&TeleopTwistJoyNode::joyCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Teleop enable button %" PRId64 ".", enable_button_);
  RCLCPP_INFO(this->get_logger(), "Turbo on button %" PRId64 ".", enable_turbo_button_);
  RCLCPP_INFO(this->get_logger(), "%s", "Teleop enable inverted reverse.");

  for (std::map<std::string, int64_t>::iterator it = axis_chassis_map_.begin();
       it != axis_chassis_map_.end(); ++it) {
    if (it->second != -1L) {
      RCLCPP_INFO(
        this->get_logger(), "Linear axis %s on %" PRId64 " at scale %f.", it->first.c_str(),
        it->second, scale_chassis_map_["normal"][it->first]);
    }
    if (enable_turbo_button_ >= 0 && it->second != -1) {
      RCLCPP_INFO(
        this->get_logger(), "Turbo for linear axis %s is scale %f.", it->first.c_str(),
        scale_chassis_map_["turbo"][it->first]);
    }
  }

  for (std::map<std::string, int64_t>::iterator it = axis_gimbal_map_.begin();
       it != axis_gimbal_map_.end(); ++it) {
    if (it->second != -1L) {
      RCLCPP_INFO(
        this->get_logger(), "Angular axis %s on %" PRId64 " at scale %f.", it->first.c_str(),
        it->second, scale_gimbal_map_["normal"][it->first]);
    }
    if (enable_turbo_button_ >= 0 && it->second != -1) {
      RCLCPP_INFO(
        this->get_logger(), "Turbo for angular axis %s is scale %f.", it->first.c_str(),
        scale_gimbal_map_["turbo"][it->first]);
    }
  }
}

double TeleopTwistJoyNode::getVal(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t> & axis_map,
  const std::map<std::string, double> & scale_map, const std::string & fieldname)
{
  auto axis_it = axis_map.find(fieldname);
  auto scale_it = scale_map.find(fieldname);

  if (
    axis_it == axis_map.end() || axis_it->second == -1L || scale_it == scale_map.end() ||
    static_cast<int>(joy_msg->axes.size()) <= axis_it->second) {
    return 0.0;
  }

  return joy_msg->axes[axis_it->second] * scale_it->second;
}

void TeleopTwistJoyNode::fillShootMsg(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map,
  example_interfaces::msg::UInt8 * shoot_msg)
{
  shoot_msg->data = getVal(joy_msg, axis_gimbal_map_, scale_gimbal_map_[which_map], "shoot");
  shoot_pub_->publish(*shoot_msg);
}

void TeleopTwistJoyNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  // Calculate the frequency of the callback function
  static auto last_time = this->now();
  auto current_time = this->now();
  dt_ = (current_time - last_time).seconds();
  last_time = current_time;

  if (
    enable_turbo_button_ >= 0 && static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button_ &&
    joy_msg->buttons[enable_turbo_button_]) {
    sendCmdVelMsg(joy_msg, "turbo");
  } else if (
    !require_enable_button_ || (static_cast<int>(joy_msg->buttons.size()) > enable_button_ &&
                                joy_msg->buttons[enable_button_])) {
    sendCmdVelMsg(joy_msg, "normal");
  } else {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (sent_disable_msg_) {
      sendZeroCommand();
      sent_disable_msg_ = false;
    }
  }
  fillShootMsg(joy_msg, "normal", new example_interfaces::msg::UInt8());
}

void TeleopTwistJoyNode::sendCmdVelMsg(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map)
{
  if (control_mode_ == "manual_control") {
    if (publish_stamped_twist_) {
      auto cmd_vel_stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      cmd_vel_stamped_msg->header.stamp = this->now();
      cmd_vel_stamped_msg->header.frame_id = robot_base_frame_;
      fillCmdVelMsg(joy_msg, which_map, &cmd_vel_stamped_msg->twist);
      cmd_vel_stamped_pub_->publish(std::move(cmd_vel_stamped_msg));
    } else {
      auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
      fillCmdVelMsg(joy_msg, which_map, cmd_vel_msg.get());
      cmd_vel_pub_->publish(std::move(cmd_vel_msg));
    }
  } else {
    sendGoalPoseAction(joy_msg, which_map);
  }
  auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  fillJointStateMsg(joy_msg, which_map, joint_state_msg.get());
  joint_state_pub_->publish(std::move(joint_state_msg));
  sent_disable_msg_ = true;
}

void TeleopTwistJoyNode::fillCmdVelMsg(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map,
  geometry_msgs::msg::Twist * cmd_vel_msg)
{
  double lin_x = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "x");
  double ang_z = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "yaw");

  cmd_vel_msg->linear.x = lin_x;
  cmd_vel_msg->linear.y = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "y");
  cmd_vel_msg->linear.z = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "z");
  cmd_vel_msg->angular.z = (lin_x < 0.0 && inverted_reverse_) ? -ang_z : ang_z;
  cmd_vel_msg->angular.y =
    getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "pitch");
  cmd_vel_msg->angular.x =
    getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "roll");
}

void TeleopTwistJoyNode::fillJointStateMsg(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map,
  sensor_msgs::msg::JointState * joint_state_msg)
{
  static double pitch = 0.0;
  static double yaw = 0.0;

  pitch += getVal(joy_msg, axis_gimbal_map_, scale_gimbal_map_[which_map], "pitch") * dt_;
  yaw += getVal(joy_msg, axis_gimbal_map_, scale_gimbal_map_[which_map], "yaw") * dt_;

  joint_state_msg->header.stamp = this->now();
  joint_state_msg->name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
  joint_state_msg->position = {pitch, yaw};
}

void TeleopTwistJoyNode::sendGoalPoseAction(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map)
{
  double x = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "x");
  double y = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "y");
  if (abs(x) <= 0.1 && abs(y) <= 0.1) {
    sent_disable_msg_ = true;
    return;
  }

  geometry_msgs::msg::PoseStamped gimbal_pose;
  gimbal_pose.pose.position.x = x;
  gimbal_pose.pose.position.y = y;

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.stamp = this->now();
  goal.pose.header.frame_id = "map";

  try {
    auto transform = tf_buffer_->lookupTransform("map", robot_base_frame_, tf2::TimePointZero);
    tf2::doTransform(gimbal_pose, goal.pose, transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Failed to transform goal pose from %s to map: %s",
      robot_base_frame_.c_str(), ex.what());
    return;
  }
  static auto last_goal_time = this->now();
  auto current_time = this->now();
  if ((current_time - last_goal_time).seconds() >= 0.25) {
    auto goal_handle_future = nav_to_pose_client_->async_send_goal(goal);
    last_goal_time = current_time;
  }
}

void TeleopTwistJoyNode::sendZeroCommand()
{
  if (control_mode_ == "auto_control") {
    auto goal_handle_future = nav_to_pose_client_->async_cancel_goals_before(this->now());
  }
  if (publish_stamped_twist_) {
    auto cmd_vel_stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd_vel_stamped_msg->header.stamp = this->now();
    cmd_vel_stamped_msg->header.frame_id = robot_base_frame_;
    cmd_vel_stamped_pub_->publish(std::move(cmd_vel_stamped_msg));
  } else {
    auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel_pub_->publish(std::move(cmd_vel_msg));
  }
}
}  // namespace pb_teleop_twist_joy

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pb_teleop_twist_joy::TeleopTwistJoyNode)
