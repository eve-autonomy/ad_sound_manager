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

#include <autoware_state_machine_msgs/msg/state_machine.hpp>
#include <chrono>
#include <thread>

using StateMachine = autoware_state_machine_msgs::msg::StateMachine;

class TestableAdSoundManager : public ad_sound_manager::AdSoundManager
{
public:
  explicit TestableAdSoundManager(const rclcpp::NodeOptions & options)
  : AdSoundManager(options) {}

  uint16_t getServiceLayerState() const { return cur_service_layer_state_; }
  uint8_t getControlLayerState() const { return cur_control_layer_state_; }
  int getOnePlayState() const { return one_play_state_; }

  static rclcpp::NodeOptions createTestNodeOptions()
  {
    rclcpp::NodeOptions node_options;
    const std::string test_sound_file = "test.wav";
    const std::string test_sound_dir = "/tmp/ad_sound_manager_test/";

    node_options.parameter_overrides({
      rclcpp::Parameter("sound_filename_avoid", test_sound_file),
      rclcpp::Parameter("sound_filename_start", test_sound_file),
      rclcpp::Parameter("sound_filename_left", test_sound_file),
      rclcpp::Parameter("sound_filename_right", test_sound_file),
      rclcpp::Parameter("sound_filename_bgm", test_sound_file),
      rclcpp::Parameter("sound_filename_obstacle", test_sound_file),
      rclcpp::Parameter("sound_filename_wakeup", test_sound_file),
      rclcpp::Parameter("sound_filename_leave", test_sound_file),
      rclcpp::Parameter("sound_filename_arrival", test_sound_file),
      rclcpp::Parameter("sound_filename_call", test_sound_file),
      rclcpp::Parameter("sound_filename_alert_imu_initialize", test_sound_file),
      rclcpp::Parameter("sound_directory_path", test_sound_dir),
    });
    return node_options;
  }
};

class AdSoundManagerStateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    system("mkdir -p /tmp/ad_sound_manager_test && touch /tmp/ad_sound_manager_test/test.wav");

    node_ = std::make_shared<TestableAdSoundManager>(
      TestableAdSoundManager::createTestNodeOptions());
    test_node_ = std::make_shared<rclcpp::Node>("test_helper_node");

    state_pub_ = test_node_->create_publisher<StateMachine>(
      "/autoware_state_machine/state", rclcpp::QoS{3}.transient_local());

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  void TearDown() override
  {
    node_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  void spinOnce(int times = 1)
  {
    for (int i = 0; i < times; ++i) {
      rclcpp::spin_some(node_);
      rclcpp::spin_some(test_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void publishState(uint16_t service_layer_state, uint8_t control_layer_state)
  {
    StateMachine msg;
    msg.service_layer_state = service_layer_state;
    msg.control_layer_state = control_layer_state;
    state_pub_->publish(msg);
    spinOnce(5);
  }

  std::shared_ptr<TestableAdSoundManager> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Publisher<StateMachine>::SharedPtr state_pub_;
};

TEST_F(AdSoundManagerStateTest, ChangeSoundStateUpdatesCurrentState)
{
  publishState(StateMachine::STATE_RUNNING, StateMachine::AUTO);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_RUNNING);
  EXPECT_EQ(node_->getControlLayerState(), StateMachine::AUTO);

  publishState(StateMachine::STATE_DURING_RECEIVE_ROUTE, StateMachine::MANUAL);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_DURING_RECEIVE_ROUTE);
  EXPECT_EQ(node_->getControlLayerState(), StateMachine::MANUAL);
}

TEST_F(AdSoundManagerStateTest, OneShotStateSetsOnePlayState)
{
  publishState(StateMachine::STATE_CHECK_NODE_ALIVE, StateMachine::MANUAL);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_CHECK_NODE_ALIVE);
  EXPECT_EQ(node_->getOnePlayState(), StateMachine::STATE_CHECK_NODE_ALIVE);
}
