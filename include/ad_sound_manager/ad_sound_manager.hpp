// Copyright 2020 eve autonomy inc. All Rights Reserved.
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
// limitations under the License

#ifndef AD_SOUND_MANAGER__AD_SOUND_MANAGER_HPP_
#define AD_SOUND_MANAGER__AD_SOUND_MANAGER_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

// Audio driver messages
// For: /sound_voice_alarm/audio_cmd, /sound_bgm/audio_cmd
#include "audio_driver_msgs/msg/sound_driver_ctrl.hpp"
// For: /sound_voice_alarm/audio_res
#include "audio_driver_msgs/msg/sound_driver_res.hpp"

// autoware_state_machine messages (to be removed after migration)
// For: /autoware_state_machine/state (legacy - to be removed)
#include "autoware_state_machine_msgs/msg/state_machine.hpp"
// For: /autoware_state_machine/state_sound_done (kept for eve_cmd_gate)
#include "autoware_state_machine_msgs/msg/state_sound_done.hpp"

// New ADAPI v1 messages
// For: /api/motion/state
#include "autoware_adapi_v1_msgs/msg/motion_state.hpp"
// For: /api/vehicle/status (turn_indicators)
#include "autoware_adapi_v1_msgs/msg/vehicle_status.hpp"
// For: /api/routing/state
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
// For: /api/localization/initialization_state
#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"
// For: /api/operation_mode/state
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"

// go_interface messages
// For: external system integration (voice_flg, lock_flg)
#include "go_interface_msgs/msg/vehicle_status.hpp"

// External API messages
// For: /api/external/get/hazard_status (emergency_holding)
#include "tier4_external_api_msgs/msg/hazard_status_stamped.hpp"

// Legacy messages (to be removed after migration)
// For: /awapi/vehicle/get/status (legacy - to be replaced by /api/vehicle/status)
#include "tier4_api_msgs/msg/awapi_vehicle_status.hpp"
// For: turn signal constants (legacy - to be replaced by TurnIndicators)
#include "tier4_vehicle_msgs/msg/turn_signal.hpp"
// TODO: Replace with /api/external/get/planning_factors for stop reason detection
// For: /awapi/autoware/get/status (stop_reason)
#include "tier4_api_msgs/msg/awapi_autoware_status.hpp"

// Other messages
// For: /localization/initial_pose/sound/response
#include "tier4_external_api_msgs/msg/response_status.hpp"
// For: /localization/initial_pose/sound/request
#include "sound_msgs/msg/sound_request.hpp"

#define VOLUME_VOICE_ALARM          (1.0)
#define VOLUME_HIGH_BGM             (0.3)
#define VOLUME_LOW_BGM              (0.2)
#define VOLUME_ZERO_BGM             (0.0)

#define LOOP_DELAY_BGM              (0.0)
#define LOOP_DELAY_VOICE_ALARM      (2.0)
#define LOOP_DELAY_VOICE_ALARM_LONG (3.0)
#define START_DELAY_NONE            (0.0)
#define START_DELAY_CHANGE          (1.0)
#define CURRENT_PLAY_DURATION       (2.0)

namespace ad_sound_manager
{
class AdSoundManager : public rclcpp::Node
{
public:
  explicit AdSoundManager(const rclcpp::NodeOptions & options);
  virtual ~AdSoundManager();

protected:
  // ============================================================
  // State variables accessible from test subclass
  // ============================================================

  // Store the latest status (service_layer_state and control_layer_state)
  uint16_t cur_service_layer_state_;
  uint16_t prev_service_layer_state_;
  uint8_t cur_control_layer_state_;
  uint8_t prev_control_layer_state_;

  // Store the status of one-time playback waiting for a response.
  int one_play_state_;

  // Sound playback flags (for stateless state determination)
  bool is_playing_wakeup_sound_ = false;
  bool is_playing_engage_sound_ = false;
  bool is_playing_restart_sound_ = false;
  bool is_playing_arrival_sound_ = false;

  // Flag to track if engage sound has completed (for stateless STATE_INSTRUCT_ENGAGE)
  // Set when engage sound completes, cleared when motion becomes MOVING
  bool engage_sound_completed_ = false;

  // Flag to track if driving has started (motion=MOVING occurred)
  // Reset when route becomes UNSET
  bool has_started_driving_ = false;

  // For: /api/motion/state
  // Values: UNKNOWN(0), STOPPED(1), STARTING(2), MOVING(3)
  autoware_adapi_v1_msgs::msg::MotionState motion_state_;
  autoware_adapi_v1_msgs::msg::MotionState prev_motion_state_;

  // For: /api/vehicle/status
  // Contains: gear, turn_indicators, hazard_lights, steering_tire_angle
  autoware_adapi_v1_msgs::msg::VehicleStatus adapi_vehicle_status_;
  autoware_adapi_v1_msgs::msg::VehicleStatus prev_adapi_vehicle_status_;

  // For: /api/routing/state
  // Values: UNKNOWN(0), UNSET(1), SET(2), ARRIVED(3), CHANGING(4)
  autoware_adapi_v1_msgs::msg::RouteState route_state_;
  autoware_adapi_v1_msgs::msg::RouteState prev_route_state_;

  // For: /api/localization/initialization_state
  // Values: UNKNOWN(0), UNINITIALIZED(1), INITIALIZING(2), INITIALIZED(3)
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState localization_state_;
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState prev_localization_state_;

  // For: /api/operation_mode/state
  // Contains: mode, is_autoware_control_enabled, is_in_transition, etc.
  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode_state_;
  autoware_adapi_v1_msgs::msg::OperationModeState prev_operation_mode_state_;

  // For: external system integration (voice_flg, lock_flg)
  go_interface_msgs::msg::VehicleStatus go_interface_vehicle_status_;
  go_interface_msgs::msg::VehicleStatus prev_go_interface_vehicle_status_;

  // For: /api/external/get/hazard_status (emergency_holding)
  bool emergency_holding_ = false;

  // Sound playback state for localization initial pose
  bool is_playing_sound_initialpose_;

private:
  enum SoundChannel
  {
    CHANNEL_VOICE = 0,
    CHANNEL_BGM
  };
  enum TurnState
  {
    NORMAL = 0,
    LEFT,
    RIGHT
  };
  enum PreSoundType
  {
    SOUND_NONE = 0,
    NORMAL_SOUND,
    TURN_LEFTRIGHT_SOUND,
    STOP_REASON_SOUND
  };

  // ============================================================
  // Publishers
  // ============================================================
  rclcpp::Publisher<audio_driver_msgs::msg::SoundDriverCtrl>::SharedPtr pub_bgm_cmd_, pub_voice_cmd_;
  rclcpp::Publisher<autoware_state_machine_msgs::msg::StateSoundDone>::SharedPtr pub_sound_done_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::ResponseStatus>::SharedPtr pub_sound_response_initialpose_;

  // ============================================================
  // Subscriptions - New ADAPI v1
  // ============================================================
  // For: /api/motion/state
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::MotionState>::SharedPtr sub_motion_state_;
  // For: /api/vehicle/status
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::VehicleStatus>::SharedPtr sub_adapi_vehicle_status_;
  // For: /api/routing/state
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_route_state_;
  // For: /api/localization/initialization_state
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr sub_localization_state_;
  // For: /api/operation_mode/state
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_operation_mode_state_;

  // ============================================================
  // Subscriptions - go_interface
  // ============================================================
  // For: external system integration (voice_flg, lock_flg)
  rclcpp::Subscription<go_interface_msgs::msg::VehicleStatus>::SharedPtr sub_go_interface_vehicle_status_;

  // ============================================================
  // Subscriptions - System
  // ============================================================
  // For: /api/external/get/hazard_status (emergency_holding)
  rclcpp::Subscription<tier4_external_api_msgs::msg::HazardStatusStamped>::SharedPtr sub_hazard_status_;

  // ============================================================
  // Subscriptions - Legacy (TODO: Replace with planning_factors)
  // ============================================================
  // TODO: Replace with /api/external/get/planning_factors
  rclcpp::Subscription<tier4_api_msgs::msg::AwapiAutowareStatus>::SharedPtr sub_awapi_autoware_status_;

  // ============================================================
  // Subscriptions - Other
  // ============================================================
  rclcpp::Subscription<audio_driver_msgs::msg::SoundDriverRes>::SharedPtr sub_bgm_res_, sub_voice_res_;
  rclcpp::Subscription<sound_msgs::msg::SoundRequest>::SharedPtr sub_sound_request_initialpose_;

  audio_driver_msgs::msg::SoundDriverCtrl sdc_msg_;

  // Turn signal information from AwapiVehicleStatus.
  int32_t turn_signal_;

  // Turning state information.
  //   - during right sound playback : RIGHT
  //   - during left sound playback  : LEFT
  //   - other                       : NORMAL
  enum TurnState turn_state_;

  // Continuos state for turning sound.
  //   - during right/left sound playback : true
  //   - other                            : false
  bool continuity_state_;

  // ============================================================
  // Legacy state variables (TODO: Replace with planning_factors)
  // ============================================================

  // For: /awapi/autoware/get/status (stop_reason)
  // TODO: Replace with /api/external/get/planning_factors
  tier4_api_msgs::msg::AwapiAutowareStatus awapi_autoware_status_;

  void makeFullPathWithFileCheck(std::string & file_path);

  // ============================================================
  // Callback functions - New ADAPI v1
  // ============================================================
  // For: /api/motion/state
  void callbackMotionState(
    const autoware_adapi_v1_msgs::msg::MotionState::ConstSharedPtr msg);
  // For: /api/vehicle/status
  void callbackAdapiVehicleStatus(
    const autoware_adapi_v1_msgs::msg::VehicleStatus::ConstSharedPtr msg);
  // For: /api/routing/state
  void callbackRouteState(
    const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg);
  // For: /api/localization/initialization_state
  void callbackLocalizationState(
    const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg);
  // For: /api/operation_mode/state
  void callbackOperationModeState(
    const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);

  // ============================================================
  // Callback functions - go_interface
  // ============================================================
  // For: external system integration (voice_flg, lock_flg)
  void callbackGoInterfaceVehicleStatus(
    const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg);

  // ============================================================
  // Callback functions - System
  // ============================================================
  // For: /api/external/get/hazard_status (emergency_holding)
  void callbackHazardStatus(
    const tier4_external_api_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);

  // ============================================================
  // Callback functions - Legacy (TODO: Replace with planning_factors)
  // ============================================================
  // TODO: Replace with /api/external/get/planning_factors
  void callbackAwapiAutowareStatus(
    const tier4_api_msgs::msg::AwapiAutowareStatus::ConstSharedPtr msg);

  // ============================================================
  // Callback functions - Other
  // ============================================================
  void callbackVoiceRes(const audio_driver_msgs::msg::SoundDriverRes::ConstSharedPtr msg);
  void callbackSoundRequestInitialpose(const sound_msgs::msg::SoundRequest::ConstSharedPtr msg);

  // ============================================================
  // State conversion function
  // ============================================================
  // Converts new ADAPI v1 states to legacy service_layer_state and control_layer_state
  void updateAutowareStateFromTopics(void);

  void publishSoundDone(void);

  const audio_driver_msgs::msg::SoundDriverCtrl initAudioCmd(
    const int cmd_type,
    const float volume = 1.0,
    const std::string file_path = "",
    const bool is_loop = true,
    const float loop_delay = 0.0,
    const float start_delay = 0.0) const;

  void playOneshotVoice(const std::string file_path, const bool cut_in = false);
  void playLoopVoice(const std::string file_path,
    const bool cut_in = false, const bool is_long_delay = false);
  void playLoopNoBGM(const std::string file_path);
  void playLoopBGM(const std::string file_path);

  PreSoundType checkPreSoundType(void);

  void changeSoundState(
    const uint16_t service_layer_state, const uint8_t control_layer_state,
    bool force);

  // Helper function to convert state to string for debug logging
  std::string serviceLayerStateToString(uint16_t state) const;
  std::string controlLayerStateToString(uint8_t state) const;
  std::string motionStateToString(uint16_t state) const;
  std::string routeStateToString(uint16_t state) const;
  std::string localizationStateToString(uint16_t state) const;

  std::string sound_filename_avoid_ = "";
  std::string sound_filename_start_ = "";
  std::string sound_filename_left_ = "";
  std::string sound_filename_right_ = "";
  std::string sound_filename_bgm_ = "";
  std::string sound_filename_obstacle_ = "";
  std::string sound_filename_wakeup_ = "";
  std::string sound_filename_leave_ = "";
  std::string sound_filename_arrival_ = "";
  std::string sound_filename_call_ = "";
  std::string sound_filename_alert_imu_initialize_ = "";
  std::string pre_sound_filename_ = "";

  std::string sound_directory_path_ = "";
};

}  // namespace ad_sound_manager
#endif  // AD_SOUND_MANAGER__AD_SOUND_MANAGER_HPP_
