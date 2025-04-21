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

TEST(AdSoundManagerTest, DT_3_4_1)
{
  rclcpp::init(0, nullptr);
  int count = 0;
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
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 2; i++) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

TEST(AdSoundManagerTest, DT_3_8)
{
  rclcpp::init(0, nullptr);
  int count = 0;
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
  for (int i = 0; i < 2; i++) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_EQ(response.code, tier4_external_api_msgs::msg::ResponseStatus::SUCCESS);
  rclcpp::shutdown();
}
