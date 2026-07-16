// Copyright 2025 Tier IV, Inc.
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

#include "ad_sound_manager/ad_sound_manager.hpp"
#include "gtest/gtest.h"
#include "geometry_msgs/msg/point.hpp"

#include <chrono>
#include <thread>

namespace
{
constexpr int kInitialSpinAttempts = 3;
constexpr int kPlaybackSpinAttempts = 5;
constexpr std::chrono::milliseconds kSpinDelay{100};
}  // namespace

class AdSoundManagerTest : public ad_sound_manager::AdSoundManager
{
public:
  AdSoundManagerTest() : AdSoundManager(setupNodeOptions()) {}
  void setIsPlayingSoundInitialpose(bool is_playing_sound_initialpose)
  {
    is_playing_sound_initialpose_ = is_playing_sound_initialpose;
  }

private:
  static rclcpp::NodeOptions setupNodeOptions()
  {
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides().push_back(rclcpp::Parameter(
      "sound_filename_avoid", rclcpp::ParameterValue("alert_imu_initialize.wav")));
    node_options.parameter_overrides().push_back(rclcpp::Parameter(
      "sound_filename_start", rclcpp::ParameterValue("alert_imu_initialize.wav")));
    node_options.parameter_overrides().push_back(
      rclcpp::Parameter("sound_filename_left", rclcpp::ParameterValue("alert_imu_initialize.wav")));
    node_options.parameter_overrides().push_back(rclcpp::Parameter(
      "sound_filename_right", rclcpp::ParameterValue("alert_imu_initialize.wav")));
    node_options.parameter_overrides().push_back(
      rclcpp::Parameter("sound_filename_bgm", rclcpp::ParameterValue("alert_imu_initialize.wav")));
    node_options.parameter_overrides().push_back(rclcpp::Parameter(
      "sound_filename_obstacle", rclcpp::ParameterValue("alert_imu_initialize.wav")));
    node_options.parameter_overrides().push_back(rclcpp::Parameter(
      "sound_filename_alert_imu_initialize", rclcpp::ParameterValue("alert_imu_initialize.wav")));
    node_options.parameter_overrides().push_back(rclcpp::Parameter(
      "sound_directory_path",
      rclcpp::ParameterValue(
        "/home/masahirokubota/eve/v4.4.0/pilot-auto.x1.eve/src/x1/dataset/ad_sound/wavs/ja")));
    return node_options;
  }
};

// DT_3_4
TEST(AdSoundManagerTest, DT_3_4_1)
{
  rclcpp::init(0, nullptr);
  audio_driver_msgs::msg::SoundDriverCtrl sound_driver_ctrl;
  auto node = std::make_shared<AdSoundManagerTest>();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto publisher = test_node->create_publisher<sound_msgs::msg::SoundRequest>(
    "/localization/initial_pose/sound/request", rclcpp::QoS{3}.transient_local());
  auto subscriber = test_node->create_subscription<audio_driver_msgs::msg::SoundDriverCtrl>(
    "/sound_voice_alarm/audio_cmd", rclcpp::QoS{5}.transient_local(),
    [&sound_driver_ctrl](const audio_driver_msgs::msg::SoundDriverCtrl msg) {
      RCLCPP_INFO(rclcpp::get_logger("test"), "Received message");
      sound_driver_ctrl = msg;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  sound_msgs::msg::SoundRequest sound_request;
  sound_request.stamp = rclcpp::Clock().now();
  sound_request.sound_type = "alert_imu_initialize";
  publisher->publish(sound_request);
  const auto timeout = std::chrono::seconds(5);
  for (int i = 0; i < kInitialSpinAttempts - 1; i++) {
    executor.spin_some();
    std::this_thread::sleep_for(kSpinDelay);
  }
  EXPECT_EQ(sound_driver_ctrl.cmd_type, audio_driver_msgs::msg::SoundDriverCtrl::CMD_PLAY);
  EXPECT_EQ(sound_driver_ctrl.volume, 1.0);
  // TODO(kubota): ファイルパスがローカルに依存しているのでどうにかする。
  EXPECT_EQ(
    sound_driver_ctrl.file_path,
    "/home/masahirokubota/eve/v4.4.0/pilot-auto.x1.eve/src/x1/dataset/ad_sound/wavs/ja/"
    "alert_imu_initialize.wav");
  EXPECT_EQ(sound_driver_ctrl.is_loop, false);
  EXPECT_EQ(sound_driver_ctrl.loop_delay, 0.0);
  EXPECT_EQ(sound_driver_ctrl.start_delay, 0.0);
  rclcpp::shutdown();
}

// DT_3_8
TEST(AdSoundManagerTest, DT_3_8_1)
{
  rclcpp::init(0, nullptr);
  tier4_external_api_msgs::msg::ResponseStatus response;
  auto node = std::make_shared<AdSoundManagerTest>();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto publisher = test_node->create_publisher<audio_driver_msgs::msg::SoundDriverRes>(
    "/sound_voice_alarm/audio_res", rclcpp::QoS{3}.transient_local());
  auto subscriber = test_node->create_subscription<tier4_external_api_msgs::msg::ResponseStatus>(
    "/localization/initial_pose/sound/response", rclcpp::QoS{5}.transient_local(),
    [&response](const tier4_external_api_msgs::msg::ResponseStatus msg) {
      RCLCPP_INFO(rclcpp::get_logger("test"), "Received message");
      response = msg;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);
  node->setIsPlayingSoundInitialpose(true);
  audio_driver_msgs::msg::SoundDriverRes response_status;
  publisher->publish(response_status);
  for (int i = 0; i < kInitialSpinAttempts - 1; i++) {
    executor.spin_some();
    std::this_thread::sleep_for(kSpinDelay);
  }
  EXPECT_EQ(response.code, tier4_external_api_msgs::msg::ResponseStatus::SUCCESS);
  rclcpp::shutdown();
}

TEST(AdSoundManagerTest, StopReasonsAreProcessedAfterStateUpdate)
{
  rclcpp::init(0, nullptr);
  int voice_cmd_count = 0;
  audio_driver_msgs::msg::SoundDriverCtrl last_voice_cmd;

  auto node = std::make_shared<AdSoundManagerTest>();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto state_pub = test_node->create_publisher<autoware_state_machine_msgs::msg::StateMachine>(
    "/autoware_state_machine/state", rclcpp::QoS{3}.transient_local());
  auto stop_reason_pub = test_node->create_publisher<tier4_planning_msgs::msg::StopReasonArray>(
    "/planning/scenario_planning/status/stop_reasons", rclcpp::QoS{3}.transient_local());
  auto voice_sub = test_node->create_subscription<audio_driver_msgs::msg::SoundDriverCtrl>(
    "/sound_voice_alarm/audio_cmd", rclcpp::QoS{5}.transient_local(),
    [&voice_cmd_count, &last_voice_cmd](const audio_driver_msgs::msg::SoundDriverCtrl msg) {
      ++voice_cmd_count;
      last_voice_cmd = msg;
    });
  (void)voice_sub;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  autoware_state_machine_msgs::msg::StateMachine state_msg;
  state_msg.service_layer_state =
    autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE;
  state_msg.control_layer_state = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  state_pub->publish(state_msg);

  tier4_planning_msgs::msg::StopReasonArray stop_reasons;
  stop_reasons.header.stamp = rclcpp::Clock().now();
  tier4_planning_msgs::msg::StopReason stop_reason;
  stop_reason.reason = tier4_planning_msgs::msg::StopReason::DETECTION_AREA;
  tier4_planning_msgs::msg::StopFactor stop_factor;
  stop_factor.stop_pose.orientation.w = 1.0;
  geometry_msgs::msg::Point stop_point;
  stop_point.x = 1.0;
  stop_point.y = 0.0;
  stop_point.z = 0.0;
  stop_factor.stop_factor_points.push_back(stop_point);
  stop_reason.stop_factors.push_back(stop_factor);
  stop_reasons.stop_reasons.push_back(stop_reason);
  stop_reason_pub->publish(stop_reasons);

  for (int i = 0; i < kPlaybackSpinAttempts; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(kSpinDelay);
  }

  EXPECT_GT(voice_cmd_count, 0);
  EXPECT_EQ(last_voice_cmd.cmd_type, audio_driver_msgs::msg::SoundDriverCtrl::CMD_PLAY);
  rclcpp::shutdown();
}

TEST(AdSoundManagerTest, EmptyStopReasonsAreBufferedUntilNonEmptyMessage)
{
  rclcpp::init(0, nullptr);
  int voice_cmd_count = 0;
  audio_driver_msgs::msg::SoundDriverCtrl last_voice_cmd;

  auto node = std::make_shared<AdSoundManagerTest>();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto state_pub = test_node->create_publisher<autoware_state_machine_msgs::msg::StateMachine>(
    "/autoware_state_machine/state", rclcpp::QoS{3}.transient_local());
  auto stop_reason_pub = test_node->create_publisher<tier4_planning_msgs::msg::StopReasonArray>(
    "/planning/scenario_planning/status/stop_reasons", rclcpp::QoS{3}.transient_local());
  auto voice_sub = test_node->create_subscription<audio_driver_msgs::msg::SoundDriverCtrl>(
    "/sound_voice_alarm/audio_cmd", rclcpp::QoS{5}.transient_local(),
    [&voice_cmd_count, &last_voice_cmd](const audio_driver_msgs::msg::SoundDriverCtrl msg) {
      ++voice_cmd_count;
      last_voice_cmd = msg;
    });
  (void)voice_sub;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  autoware_state_machine_msgs::msg::StateMachine state_msg;
  state_msg.service_layer_state =
    autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE;
  state_msg.control_layer_state = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  state_pub->publish(state_msg);

  tier4_planning_msgs::msg::StopReasonArray empty_stop_reasons;
  stop_reason_pub->publish(empty_stop_reasons);

  for (int i = 0; i < kInitialSpinAttempts; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(kSpinDelay);
  }

  EXPECT_EQ(voice_cmd_count, 0);

  tier4_planning_msgs::msg::StopReasonArray stop_reasons;
  stop_reasons.header.stamp = rclcpp::Clock().now();
  tier4_planning_msgs::msg::StopReason stop_reason;
  stop_reason.reason = tier4_planning_msgs::msg::StopReason::DETECTION_AREA;
  tier4_planning_msgs::msg::StopFactor stop_factor;
  stop_factor.stop_pose.orientation.w = 1.0;
  geometry_msgs::msg::Point stop_point;
  stop_point.x = 1.0;
  stop_point.y = 0.0;
  stop_point.z = 0.0;
  stop_factor.stop_factor_points.push_back(stop_point);
  stop_reason.stop_factors.push_back(stop_factor);
  stop_reasons.stop_reasons.push_back(stop_reason);
  stop_reason_pub->publish(stop_reasons);

  for (int i = 0; i < kPlaybackSpinAttempts; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(kSpinDelay);
  }

  EXPECT_GT(voice_cmd_count, 0);
  EXPECT_EQ(last_voice_cmd.cmd_type, audio_driver_msgs::msg::SoundDriverCtrl::CMD_PLAY);
  rclcpp::shutdown();
}
