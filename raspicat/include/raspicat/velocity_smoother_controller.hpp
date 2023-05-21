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

#ifndef RASPICAT_BRINGUP__VELOCITY_SMOOTHER_CONTROLLER_HPP_
#define RASPICAT_BRINGUP__VELOCITY_SMOOTHER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace raspicat_bringup
{

class VelocitySmootherController : public rclcpp::Node
{
public:
  VelocitySmootherController();

protected:
  void initPubSub();
  void initTimer();
  void initLifeCycleClient();
  void setParam();
  void getParam();
  void changeLifeCycleState(
    std::uint8_t transition,
    std::chrono::seconds time_out);
  template<typename FutureT, typename WaitTimeT>
  void wait_for_result(FutureT & future, WaitTimeT time_to_wait);

  void callbackJoy(sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void callbackInputVel(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void callbackCmdVelSmoothed(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void callbackControlVelPubTimer();

  void accel_decel();
  void no_accel_decel();
  void stop();
  bool shouldExecute(
    std::vector<int> button_num,
    std::vector<int64_t> button_num_param);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_smoothed_sub_;
  rclcpp::TimerBase::SharedPtr control_vel_pub_timer_;

  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>
  client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
  client_change_state_;

  sensor_msgs::msg::Joy joy_;
  geometry_msgs::msg::Twist input_vel_;
  geometry_msgs::msg::Twist cmd_vel_smoothed_;

  std::vector<int64_t> accel_decel_button_;
  std::vector<int64_t> no_accel_decel_button_;
  bool input_vel_cb_flag_;
};

} // namespace raspicat_bringup

#endif // RASPICAT_BRINGUP__VELOCITY_SMOOTHER_CONTROLLER_HPP_
