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

#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/vehicle_status.hpp>
#include <autoware_adapi_v1_msgs/msg/turn_indicators.hpp>
#include <autoware_state_machine_msgs/msg/state_machine.hpp>
#include <tier4_external_api_msgs/msg/hazard_status_stamped.hpp>
#include <audio_driver_msgs/msg/sound_driver_res.hpp>

using MotionState = autoware_adapi_v1_msgs::msg::MotionState;
using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
using LocalizationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using VehicleStatus = autoware_adapi_v1_msgs::msg::VehicleStatus;
using TurnIndicators = autoware_adapi_v1_msgs::msg::TurnIndicators;
using StateMachine = autoware_state_machine_msgs::msg::StateMachine;
using HazardStatusStamped = tier4_external_api_msgs::msg::HazardStatusStamped;
using SoundDriverRes = audio_driver_msgs::msg::SoundDriverRes;

// ============================================================
// Testable subclass with Getter accessors
// ============================================================
class TestableAdSoundManager : public ad_sound_manager::AdSoundManager
{
public:
  explicit TestableAdSoundManager(const rclcpp::NodeOptions & options)
  : AdSoundManager(options) {}

  // === ServiceLayerState / ControlLayerState ===
  uint16_t getServiceLayerState() const { return cur_service_layer_state_; }
  uint8_t getControlLayerState() const { return cur_control_layer_state_; }
  uint16_t getPrevServiceLayerState() const { return prev_service_layer_state_; }
  uint8_t getPrevControlLayerState() const { return prev_control_layer_state_; }

  // === One-shot playback state ===
  int getOnePlayState() const { return one_play_state_; }

  // === Sound playback flags ===
  bool isPlayingWakeupSound() const { return is_playing_wakeup_sound_; }
  bool isPlayingEngageSound() const { return is_playing_engage_sound_; }
  bool isPlayingRestartSound() const { return is_playing_restart_sound_; }
  bool isPlayingArrivalSound() const { return is_playing_arrival_sound_; }

  // === Internal flags ===
  bool hasStartedDriving() const { return has_started_driving_; }
  bool isEngageSoundCompleted() const { return engage_sound_completed_; }

  // === Topic states ===
  uint16_t getMotionState() const { return motion_state_.state; }
  uint16_t getRouteState() const { return route_state_.state; }
  uint16_t getLocalizationState() const { return localization_state_.state; }
  bool isAutowareControlEnabled() const { return operation_mode_state_.is_autoware_control_enabled; }
  bool isEmergencyHolding() const { return emergency_holding_; }

  // === Setters for testing (protected member access) ===
  void setIsPlayingSoundInitialpose(bool is_playing_sound_initialpose)
  {
    is_playing_sound_initialpose_ = is_playing_sound_initialpose;
  }

  // === Static helper to create NodeOptions with required parameters ===
  static rclcpp::NodeOptions createTestNodeOptions()
  {
    rclcpp::NodeOptions node_options;
    // Use a test directory that exists or create a minimal test setup
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

// ============================================================
// Legacy test helper class (for backward compatibility)
// ============================================================
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
    // Use environment-independent test directory (same as TestableAdSoundManager)
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

// ============================================================
// Test Fixture for State Transition Tests
// ============================================================
class StateTransitionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Create test sound directory
    system("mkdir -p /tmp/ad_sound_manager_test && touch /tmp/ad_sound_manager_test/test.wav");

    // Create test target node
    node_ = std::make_shared<TestableAdSoundManager>(
      TestableAdSoundManager::createTestNodeOptions());

    // Create test helper node for publishing
    test_node_ = std::make_shared<rclcpp::Node>("test_helper_node");

    // Create publishers for ADAPI topics (use transient_local to match subscribers)
    motion_pub_ = test_node_->create_publisher<MotionState>(
      "/api/motion/state", rclcpp::QoS{1}.transient_local());
    route_pub_ = test_node_->create_publisher<RouteState>(
      "/api/routing/state", rclcpp::QoS{1}.transient_local());
    localization_pub_ = test_node_->create_publisher<LocalizationState>(
      "/api/localization/initialization_state", rclcpp::QoS{1}.transient_local());
    operation_mode_pub_ = test_node_->create_publisher<OperationModeState>(
      "/api/operation_mode/state", rclcpp::QoS{1}.transient_local());
    vehicle_status_pub_ = test_node_->create_publisher<VehicleStatus>(
      "/api/vehicle/status", rclcpp::SensorDataQoS());
    sound_res_pub_ = test_node_->create_publisher<SoundDriverRes>(
      "/sound_voice_alarm/audio_res", rclcpp::QoS{3}.transient_local());
    hazard_status_pub_ = test_node_->create_publisher<HazardStatusStamped>(
      "/api/external/get/hazard_status", rclcpp::QoS{1});

    // Wait for connections to establish
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Initial spin to process any pending callbacks
    spinOnce(5);
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

  void publishMotion(uint16_t state)
  {
    auto msg = MotionState();
    msg.state = state;
    motion_pub_->publish(msg);
    spinOnce(3);
  }

  void publishRoute(uint16_t state)
  {
    auto msg = RouteState();
    msg.state = state;
    route_pub_->publish(msg);
    spinOnce(3);
  }

  void publishLocalization(uint16_t state)
  {
    auto msg = LocalizationState();
    msg.state = state;
    localization_pub_->publish(msg);
    spinOnce(3);
  }

  void publishOperationMode(bool is_autoware_control_enabled)
  {
    auto msg = OperationModeState();
    msg.is_autoware_control_enabled = is_autoware_control_enabled;
    operation_mode_pub_->publish(msg);
    spinOnce(3);
  }

  void publishSoundResponse()
  {
    auto msg = SoundDriverRes();
    sound_res_pub_->publish(msg);
    spinOnce(10);  // More spins to ensure callback is processed
  }

  void publishHazardStatus(bool emergency_holding)
  {
    auto msg = HazardStatusStamped();
    msg.status.emergency_holding = emergency_holding;
    hazard_status_pub_->publish(msg);
    spinOnce(3);
  }

  void publishVehicleStatus(uint8_t turn_indicators)
  {
    auto msg = VehicleStatus();
    msg.turn_indicators.status = turn_indicators;
    vehicle_status_pub_->publish(msg);
    spinOnce(3);
  }

  std::shared_ptr<TestableAdSoundManager> node_;
  std::shared_ptr<rclcpp::Node> test_node_;

  rclcpp::Publisher<MotionState>::SharedPtr motion_pub_;
  rclcpp::Publisher<RouteState>::SharedPtr route_pub_;
  rclcpp::Publisher<LocalizationState>::SharedPtr localization_pub_;
  rclcpp::Publisher<OperationModeState>::SharedPtr operation_mode_pub_;
  rclcpp::Publisher<VehicleStatus>::SharedPtr vehicle_status_pub_;
  rclcpp::Publisher<SoundDriverRes>::SharedPtr sound_res_pub_;
  rclcpp::Publisher<HazardStatusStamped>::SharedPtr hazard_status_pub_;
};

// ============================================================
// State Transition Tests
// ============================================================

// Test: Initial state should be STATE_CHECK_NODE_ALIVE
TEST_F(StateTransitionTest, InitialStateIsCheckNodeAlive)
{
  spinOnce(5);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_CHECK_NODE_ALIVE);
}

// Test: After wakeup sound completed, transition to STATE_DURING_WAKEUP
TEST_F(StateTransitionTest, AfterWakeupSoundCompleted_TransitionToDuringWakeup)
{
  spinOnce(5);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_CHECK_NODE_ALIVE);

  // Simulate wakeup sound completion
  publishSoundResponse();

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_DURING_WAKEUP);
}

// Test: After localization initialized, transition to STATE_DURING_RECEIVE_ROUTE
TEST_F(StateTransitionTest, AfterLocalizationInitialized_TransitionToDuringReceiveRoute)
{
  // Complete wakeup sequence
  spinOnce(5);
  publishSoundResponse();  // Wakeup sound completed
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_DURING_WAKEUP);

  // Localization initialized, route not set
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::UNSET);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_DURING_RECEIVE_ROUTE);
}

// Test: After route set, transition to STATE_WAITING_ENGAGE_INSTRUCTION
TEST_F(StateTransitionTest, AfterRouteSet_TransitionToWaitingEngageInstruction)
{
  // Complete startup sequence
  spinOnce(5);
  publishSoundResponse();  // Wakeup sound completed
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::UNSET);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_DURING_RECEIVE_ROUTE);

  // Route set, motion stopped
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION);
  EXPECT_FALSE(node_->hasStartedDriving());
}

// Test: After motion=STARTING, transition to STATE_INFORM_ENGAGE
TEST_F(StateTransitionTest, AfterEngageStarted_TransitionToInformEngage)
{
  // Complete startup to WAITING_ENGAGE_INSTRUCTION
  spinOnce(5);
  publishSoundResponse();
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION);

  // Engage started
  publishMotion(MotionState::STARTING);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_INFORM_ENGAGE);
}

// Test: After engage sound completed, transition to STATE_INSTRUCT_ENGAGE
TEST_F(StateTransitionTest, AfterEngageSoundCompleted_TransitionToInstructEngage)
{
  // Complete startup to INFORM_ENGAGE
  spinOnce(5);
  publishSoundResponse();
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);
  publishMotion(MotionState::STARTING);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_INFORM_ENGAGE);
  EXPECT_FALSE(node_->isEngageSoundCompleted());

  // Engage sound completed
  publishSoundResponse();

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_INSTRUCT_ENGAGE);
  EXPECT_TRUE(node_->isEngageSoundCompleted());  // Flag should be set
}

// Test: After motion=MOVING, transition to STATE_RUNNING
TEST_F(StateTransitionTest, AfterMoving_TransitionToRunning)
{
  // Complete startup to INSTRUCT_ENGAGE
  spinOnce(5);
  publishSoundResponse();
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);
  publishMotion(MotionState::STARTING);
  publishSoundResponse();  // Engage sound completed
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_INSTRUCT_ENGAGE);
  EXPECT_TRUE(node_->isEngageSoundCompleted());  // Verify flag is set

  // Moving
  publishMotion(MotionState::MOVING);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_RUNNING);
  EXPECT_TRUE(node_->hasStartedDriving());
  EXPECT_FALSE(node_->isEngageSoundCompleted());  // Flag should be cleared on MOVING
}

// Test: After route=ARRIVED, transition to STATE_ARRIVED_GOAL
TEST_F(StateTransitionTest, AfterRouteArrived_TransitionToArrivedGoal)
{
  // Complete startup to RUNNING
  spinOnce(5);
  publishSoundResponse();
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);
  publishMotion(MotionState::STARTING);
  publishSoundResponse();
  publishMotion(MotionState::MOVING);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_RUNNING);

  // Arrived at goal
  publishRoute(RouteState::ARRIVED);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_ARRIVED_GOAL);
}

// ============================================================
// Stop and Restart Tests
// ============================================================

// Test: After stopped while driving, transition to STOP_DUETO_TRAFFIC_CONDITION
TEST_F(StateTransitionTest, AfterStoppedWhileDriving_TransitionToStopDuetoTrafficCondition)
{
  // Complete startup to RUNNING
  spinOnce(5);
  publishSoundResponse();
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);
  publishMotion(MotionState::STARTING);
  publishSoundResponse();
  publishMotion(MotionState::MOVING);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_RUNNING);
  EXPECT_TRUE(node_->hasStartedDriving());

  // Stop while driving (e.g., stop line)
  publishMotion(MotionState::STOPPED);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION);
}

// Test: After restart from stop, transition to INFORM_RESTART then RUNNING
TEST_F(StateTransitionTest, AfterRestartFromStop_TransitionToInformRestartThenRunning)
{
  // Complete startup to RUNNING, then stop
  spinOnce(5);
  publishSoundResponse();
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);
  publishMotion(MotionState::STARTING);
  publishSoundResponse();
  publishMotion(MotionState::MOVING);
  publishMotion(MotionState::STOPPED);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION);

  // Restart (motion=STARTING while has_started_driving=true)
  publishMotion(MotionState::STARTING);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_INFORM_RESTART);
  EXPECT_TRUE(node_->isPlayingRestartSound());

  // Restart sound completed
  publishSoundResponse();

  // Moving again
  publishMotion(MotionState::MOVING);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_RUNNING);
}

// ============================================================
// Emergency Stop Tests
// ============================================================

// Test: Emergency holding during CHECK_NODE_ALIVE should NOT transition to EMERGENCY_STOP
TEST_F(StateTransitionTest, EmergencyDuringWakeupSound_ShouldNotTransitionToEmergencyStop)
{
  spinOnce(5);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_CHECK_NODE_ALIVE);

  // Set emergency_holding = true during wakeup sound
  publishHazardStatus(true);

  // Should remain CHECK_NODE_ALIVE
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_CHECK_NODE_ALIVE);
}

// Test: Emergency holding during DURING_WAKEUP should NOT transition to EMERGENCY_STOP
TEST_F(StateTransitionTest, EmergencyDuringWakeup_ShouldNotTransitionToEmergencyStop)
{
  spinOnce(5);
  publishSoundResponse();  // Complete wakeup sound -> DURING_WAKEUP
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_DURING_WAKEUP);

  // Set emergency_holding = true during DURING_WAKEUP
  publishHazardStatus(true);

  // Should remain DURING_WAKEUP (not transition to EMERGENCY_STOP)
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_DURING_WAKEUP);
}

// Test: Emergency holding after wakeup phase should transition to EMERGENCY_STOP
TEST_F(StateTransitionTest, EmergencyAfterWakeup_ShouldTransitionToEmergencyStop)
{
  // Complete wakeup phase
  spinOnce(5);
  publishSoundResponse();  // Complete wakeup sound
  publishLocalization(LocalizationState::INITIALIZED);  // Complete localization
  publishRoute(RouteState::UNSET);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_DURING_RECEIVE_ROUTE);

  // Set emergency_holding = true after wakeup phase
  publishHazardStatus(true);

  // Should transition to EMERGENCY_STOP
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_EMERGENCY_STOP);
}

// ============================================================
// Edge Case Tests
// ============================================================

// Test: motion=MOVING at startup should not change state from CHECK_NODE_ALIVE
TEST_F(StateTransitionTest, EdgeCase_MovingAtStartup_ShouldNotChangeState)
{
  spinOnce(5);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_CHECK_NODE_ALIVE);

  // Motion=MOVING at startup (AdapiPauseInterface issue)
  publishMotion(MotionState::MOVING);

  // State should remain CHECK_NODE_ALIVE
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_CHECK_NODE_ALIVE);
}

// Test: has_started_driving flag should be reset when route becomes UNSET
TEST_F(StateTransitionTest, HasStartedDrivingFlag_ResetWhenRouteUnset)
{
  // Complete startup to RUNNING
  spinOnce(5);
  publishSoundResponse();
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);
  publishMotion(MotionState::STARTING);
  publishSoundResponse();
  publishMotion(MotionState::MOVING);
  EXPECT_TRUE(node_->hasStartedDriving());

  // Route becomes UNSET (arrived, then cleared)
  publishRoute(RouteState::ARRIVED);
  publishSoundResponse();  // Arrival sound completed
  publishRoute(RouteState::UNSET);

  EXPECT_FALSE(node_->hasStartedDriving());
}

// ============================================================
// Turn Indicator Tests (Right/Left Turn Sound)
// ============================================================

// Test: When turn_indicators=RIGHT during MOVING, transition to STATE_TURNING_RIGHT
TEST_F(StateTransitionTest, TurnIndicatorsRight_TransitionToTurningRight)
{
  // Complete startup to RUNNING
  spinOnce(5);
  publishSoundResponse();
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);
  publishMotion(MotionState::STARTING);
  publishSoundResponse();
  publishMotion(MotionState::MOVING);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_RUNNING);

  // Right turn indicator
  publishVehicleStatus(TurnIndicators::RIGHT);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_TURNING_RIGHT);
}

// Test: When turn_indicators=LEFT during MOVING, transition to STATE_TURNING_LEFT
TEST_F(StateTransitionTest, TurnIndicatorsLeft_TransitionToTurningLeft)
{
  // Complete startup to RUNNING
  spinOnce(5);
  publishSoundResponse();
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);
  publishMotion(MotionState::STARTING);
  publishSoundResponse();
  publishMotion(MotionState::MOVING);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_RUNNING);

  // Left turn indicator
  publishVehicleStatus(TurnIndicators::LEFT);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_TURNING_LEFT);
}

// Test: When turn_indicators changes from RIGHT to DISABLE, transition back to STATE_RUNNING
TEST_F(StateTransitionTest, TurnIndicatorsDisable_TransitionBackToRunning)
{
  // Complete startup to RUNNING, then turn right
  spinOnce(5);
  publishSoundResponse();
  publishLocalization(LocalizationState::INITIALIZED);
  publishRoute(RouteState::SET);
  publishMotion(MotionState::STOPPED);
  publishMotion(MotionState::STARTING);
  publishSoundResponse();
  publishMotion(MotionState::MOVING);
  publishVehicleStatus(TurnIndicators::RIGHT);
  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_TURNING_RIGHT);

  // Turn indicator off
  publishVehicleStatus(TurnIndicators::DISABLE);

  EXPECT_EQ(node_->getServiceLayerState(), StateMachine::STATE_RUNNING);
}

// ============================================================
// Legacy Tests (backward compatibility)
// ============================================================

// DT_3_4
TEST(AdSoundManagerTest, DT_3_4_1)
{
  // Create test directory and file
  system("mkdir -p /tmp/ad_sound_manager_test && touch /tmp/ad_sound_manager_test/test.wav");

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
  for (int i = 0; i < 2; i++) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_EQ(sound_driver_ctrl.cmd_type, audio_driver_msgs::msg::SoundDriverCtrl::CMD_PLAY);
  EXPECT_EQ(sound_driver_ctrl.volume, 1.0);
  // Use environment-independent test path
  EXPECT_EQ(sound_driver_ctrl.file_path, "/tmp/ad_sound_manager_test/test.wav");
  EXPECT_EQ(sound_driver_ctrl.is_loop, false);
  EXPECT_EQ(sound_driver_ctrl.loop_delay, 0.0);
  EXPECT_EQ(sound_driver_ctrl.start_delay, 0.0);
  rclcpp::shutdown();
}

// DT_3_8
TEST(AdSoundManagerTest, DT_3_8_1)
{
  // Create test directory and file
  system("mkdir -p /tmp/ad_sound_manager_test && touch /tmp/ad_sound_manager_test/test.wav");

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

  // First, complete the wakeup sound (triggered at node startup)
  audio_driver_msgs::msg::SoundDriverRes wakeup_response;
  publisher->publish(wakeup_response);
  for (int i = 0; i < 2; i++) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Now set the initialpose sound flag and publish another response
  node->setIsPlayingSoundInitialpose(true);
  audio_driver_msgs::msg::SoundDriverRes initialpose_response;
  publisher->publish(initialpose_response);
  for (int i = 0; i < 2; i++) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_EQ(response.code, tier4_external_api_msgs::msg::ResponseStatus::SUCCESS);
  rclcpp::shutdown();
}
