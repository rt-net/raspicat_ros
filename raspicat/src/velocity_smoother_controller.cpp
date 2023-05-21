// Copyright 2023 RT Corporation
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

#include <chrono>
#include <iostream>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "raspicat/velocity_smoother_controller.hpp"

using namespace std::chrono_literals;

namespace raspicat_bringup
{

VelocitySmootherController::VelocitySmootherController()
: Node("velocity_smoother_controller_node"),
  input_vel_(geometry_msgs::msg::Twist()),
  cmd_vel_smoothed_(geometry_msgs::msg::Twist()),
  input_vel_cb_flag_(false)
{
  setParam();
  getParam();
  initPubSub();
  initTimer();
  initLifeCycleClient();
  changeLifeCycleState(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
    1s);
  changeLifeCycleState(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
    1s);
}

void VelocitySmootherController::initPubSub()
{
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  control_vel_pub_ =
    create_publisher<geometry_msgs::msg::Twist>("control_vel", 10);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(
      &VelocitySmootherController::callbackJoy, this,
      std::placeholders::_1));
  input_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "input_vel", 10,
    std::bind(
      &VelocitySmootherController::callbackInputVel, this,
      std::placeholders::_1));
  cmd_vel_smoothed_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_smoothed", 10,
    std::bind(
      &VelocitySmootherController::callbackCmdVelSmoothed, this,
      std::placeholders::_1));
}

void VelocitySmootherController::initTimer()
{
  if (not strcmp(input_vel_sub_->get_topic_name(), "/key_vel")) {
    control_vel_pub_timer_ = create_wall_timer(
      500ms,
      std::bind(
        &VelocitySmootherController::callbackControlVelPubTimer,
        this));
  }
}

void VelocitySmootherController::initLifeCycleClient()
{
  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
    "velocity_smoother_node/get_state");
  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "velocity_smoother_node/change_state");
}

void VelocitySmootherController::setParam()
{
  declare_parameter("accel_decel_button", std::vector<int64_t>({2}));
  declare_parameter("no_accel_decel_button", std::vector<int64_t>({0}));
}

void VelocitySmootherController::getParam()
{
  accel_decel_button_ = get_parameter("accel_decel_button").as_integer_array();
  no_accel_decel_button_ =
    get_parameter("no_accel_decel_button").as_integer_array();
}

void VelocitySmootherController::changeLifeCycleState(
  std::uint8_t transition, std::chrono::seconds time_out)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  if (!client_change_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
      get_logger(), "Service %s is not available.",
      client_change_state_->get_service_name());
    return;
  }

  auto future_result =
    client_change_state_->async_send_request(request).future.share();
  wait_for_result(future_result, time_out);
}

template<typename FutureT, typename WaitTimeT>
void VelocitySmootherController::wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      break;
    }
    status =
      future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
}

void VelocitySmootherController::callbackJoy(
  sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  if (input_vel_cb_flag_) {
    if (shouldExecute(msg->buttons, accel_decel_button_)) {
      accel_decel();
    } else if (shouldExecute(msg->buttons, no_accel_decel_button_)) {
      no_accel_decel();
    } else {
      stop();
    }
  }
}

void VelocitySmootherController::callbackInputVel(
  geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  input_vel_cb_flag_ = true;
  input_vel_ = *msg;
}

void VelocitySmootherController::callbackCmdVelSmoothed(
  geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  cmd_vel_smoothed_ = *msg;

  if (not strcmp(input_vel_sub_->get_topic_name(), "/key_vel")) {
    accel_decel();
    control_vel_pub_timer_->reset();
  }
}

void VelocitySmootherController::callbackControlVelPubTimer() {stop();}

void VelocitySmootherController::accel_decel()
{
  control_vel_pub_->publish(input_vel_);
  cmd_vel_pub_->publish(cmd_vel_smoothed_);
}

void VelocitySmootherController::no_accel_decel()
{
  control_vel_pub_->publish(input_vel_);
  cmd_vel_pub_->publish(input_vel_);
}

void VelocitySmootherController::stop()
{
  control_vel_pub_->publish(input_vel_);
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

bool VelocitySmootherController::shouldExecute(
  std::vector<int> button_num, std::vector<int64_t> button_num_param)
{
  if (button_num_param.empty()) {
    return false;
  }

  for (auto bnp : button_num_param) {
    if (button_num[bnp]) {
      return true;
    }
  }

  return false;
}

} // namespace raspicat_bringup

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<raspicat_bringup::VelocitySmootherController>());
  rclcpp::shutdown();
  return 0;
}
