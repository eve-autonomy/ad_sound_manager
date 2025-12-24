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

#include <string>
#include <memory>
#include <utility>
#include <fstream>

#include "ad_sound_manager/ad_sound_manager.hpp"

namespace ad_sound_manager
{

AdSoundManager::AdSoundManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("ad_sound_manager", options)
{
  // ============================================================
  // Subscriptions - New ADAPI v1
  // ============================================================

  // For: /api/motion/state
  sub_motion_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::MotionState>(
    "/api/motion/state",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&AdSoundManager::callbackMotionState, this, std::placeholders::_1)
  );

  // For: /api/vehicle/status
  sub_adapi_vehicle_status_ = this->create_subscription<autoware_adapi_v1_msgs::msg::VehicleStatus>(
    "/api/vehicle/status",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&AdSoundManager::callbackAdapiVehicleStatus, this, std::placeholders::_1)
  );

  // For: /api/routing/state
  sub_route_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&AdSoundManager::callbackRouteState, this, std::placeholders::_1)
  );

  // For: /api/localization/initialization_state
  sub_localization_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
    "/api/localization/initialization_state",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&AdSoundManager::callbackLocalizationState, this, std::placeholders::_1)
  );

  // For: /api/operation_mode/state
  sub_operation_mode_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&AdSoundManager::callbackOperationModeState, this, std::placeholders::_1)
  );

  // ============================================================
  // Subscriptions - go_interface
  // ============================================================

  // For: external system integration (voice_flg, lock_flg)
  sub_go_interface_vehicle_status_ = this->create_subscription<go_interface_msgs::msg::VehicleStatus>(
    "/api_vehicle_status",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&AdSoundManager::callbackGoInterfaceVehicleStatus, this, std::placeholders::_1)
  );

  // ============================================================
  // Subscriptions - Legacy (TODO: Replace with planning_factors)
  // ============================================================

  // TODO: Replace with /api/external/get/planning_factors
  sub_awapi_autoware_status_ = this->create_subscription<tier4_api_msgs::msg::AwapiAutowareStatus>(
    "/awapi/autoware/get/status",
    rclcpp::QoS{1}.transient_local(),
    std::bind(&AdSoundManager::callbackAwapiAutowareStatus, this, std::placeholders::_1)
  );

  // ============================================================
  // Subscriptions - Other
  // ============================================================

  sub_voice_res_ = this->create_subscription<audio_driver_msgs::msg::SoundDriverRes>(
    "/sound_voice_alarm/audio_res",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::callbackVoiceRes, this, std::placeholders::_1)
  );

  sub_sound_request_initialpose_ = this->create_subscription<sound_msgs::msg::SoundRequest>(
    "/localization/initial_pose/sound/request",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::callbackSoundRequestInitialpose, this, std::placeholders::_1)
  );

  pub_voice_cmd_ = this->create_publisher<audio_driver_msgs::msg::SoundDriverCtrl>(
    "/sound_voice_alarm/audio_cmd", rclcpp::QoS{5}.transient_local());

  pub_bgm_cmd_ = this->create_publisher<audio_driver_msgs::msg::SoundDriverCtrl>(
    "/sound_bgm/audio_cmd", rclcpp::QoS{5}.transient_local());

  pub_sound_done_ =
    this->create_publisher<autoware_state_machine_msgs::msg::StateSoundDone>(
    "/autoware_state_machine/state_sound_done", rclcpp::QoS{3}.transient_local());

  pub_sound_response_initialpose_ = this->create_publisher<tier4_external_api_msgs::msg::ResponseStatus>(
    "/localization/initial_pose/sound/response", rclcpp::QoS{3}.transient_local());

  sound_filename_avoid_ = this->declare_parameter<std::string>("sound_filename_avoid", "");
  sound_filename_start_ = this->declare_parameter<std::string>("sound_filename_start", "");
  sound_filename_left_ = this->declare_parameter<std::string>("sound_filename_left", "");
  sound_filename_right_ = this->declare_parameter<std::string>("sound_filename_right", "");
  sound_filename_bgm_ = this->declare_parameter<std::string>("sound_filename_bgm", "");
  sound_filename_obstacle_ = this->declare_parameter<std::string>("sound_filename_obstacle", "");
  sound_filename_wakeup_ = this->declare_parameter<std::string>("sound_filename_wakeup", "");
  sound_filename_leave_ = this->declare_parameter<std::string>("sound_filename_leave", "");
  sound_filename_arrival_ = this->declare_parameter<std::string>("sound_filename_arrival", "");
  sound_filename_call_ = this->declare_parameter<std::string>("sound_filename_call", "");
  sound_filename_alert_imu_initialize_ = this->declare_parameter<std::string>("sound_filename_alert_imu_initialize", "");
  sound_directory_path_ = this->declare_parameter<std::string>("sound_directory_path", "");

  // Check for the audio file names.
  if ((sound_filename_avoid_ == "") ||
    (sound_filename_start_ == "") ||
    (sound_filename_left_ == "") ||
    (sound_filename_right_ == "") ||
    (sound_filename_bgm_ == "") ||
    (sound_filename_obstacle_ == ""))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "[AdSoundManager::AdSoundManager]invalid file path exists");
  }

  // Check for the audio file path name.
  if (sound_directory_path_ == "") {
    RCLCPP_ERROR(
      this->get_logger(),
      "[AdSoundManager::AdSoundManager]No current path exists");
  }

  turn_signal_ = tier4_vehicle_msgs::msg::TurnSignal::NONE;
  turn_state_ = NORMAL;
  continuity_state_ = false;
  one_play_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  cur_service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  prev_service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  cur_control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  prev_control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  is_playing_sound_initialpose_ = false;

  std::string sound_directory_path =
    sound_directory_path_.insert(sound_directory_path_.size(), "/");

  sound_filename_avoid_ = sound_directory_path + sound_filename_avoid_;
  sound_filename_start_ = sound_directory_path + sound_filename_start_;
  sound_filename_left_ = sound_directory_path + sound_filename_left_;
  sound_filename_right_ = sound_directory_path + sound_filename_right_;
  sound_filename_bgm_ = sound_directory_path + sound_filename_bgm_;
  sound_filename_obstacle_ = sound_directory_path + sound_filename_obstacle_;
  sound_filename_wakeup_ = sound_directory_path + sound_filename_wakeup_;
  sound_filename_leave_ = sound_directory_path + sound_filename_leave_;
  sound_filename_arrival_ = sound_directory_path + sound_filename_arrival_;
  sound_filename_call_ = sound_directory_path + sound_filename_call_;
  sound_filename_alert_imu_initialize_ = sound_directory_path + sound_filename_alert_imu_initialize_;

  // Check for the existence of audio files.
  makeFullPathWithFileCheck(sound_filename_avoid_);
  makeFullPathWithFileCheck(sound_filename_start_);
  makeFullPathWithFileCheck(sound_filename_left_);
  makeFullPathWithFileCheck(sound_filename_right_);
  makeFullPathWithFileCheck(sound_filename_bgm_);
  makeFullPathWithFileCheck(sound_filename_obstacle_);
  makeFullPathWithFileCheck(sound_filename_wakeup_);
  makeFullPathWithFileCheck(sound_filename_leave_);
  makeFullPathWithFileCheck(sound_filename_arrival_);
  makeFullPathWithFileCheck(sound_filename_call_);
  makeFullPathWithFileCheck(sound_filename_alert_imu_initialize_);
}

AdSoundManager::~AdSoundManager()
{
}

void AdSoundManager::makeFullPathWithFileCheck(std::string & file_path)
{
  std::ifstream ifs(file_path);

  if (!ifs.is_open()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[AdSoundManager::play]invalid file:%s", file_path.c_str());
    return;
  }

  char * full_path = realpath(file_path.c_str(), NULL);
  if (full_path == NULL) {
    RCLCPP_FATAL(
      this->get_logger(),
      "[AdSoundManager::play]realpath error");
    return;
  }

  file_path = full_path;
}

void AdSoundManager::publishSoundDone(void)
{
  autoware_state_machine_msgs::msg::StateSoundDone done_msg;
  done_msg.stamp = this->now();
  done_msg.state = one_play_state_;
  done_msg.done = true;
  pub_sound_done_->publish(done_msg);
  one_play_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
}

// callbackAutowareStateMachine() removed - replaced by updateAutowareStateFromTopics()

void AdSoundManager::callbackVoiceRes(
  const audio_driver_msgs::msg::SoundDriverRes::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::callbackVoiceRes]callbackVoiceRes");

  // Done information is returned only after one-time playback.
  if (one_play_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL)
  {
    RCLCPP_INFO(this->get_logger(),
      "[DEBUG] Arrival sound completed, transitioning to STATE_DURING_RECEIVE_ROUTE");
    publishSoundDone();
    // 到着音声再生完了後、STATE_DURING_RECEIVE_ROUTE に遷移（BGM停止）
    changeSoundState(
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE,
      cur_control_layer_state_,
      false);
    return;
  }

  if ( (one_play_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE) ||
    (one_play_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE) ||
    (one_play_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART) )
  {
    publishSoundDone();
  }
  if (is_playing_sound_initialpose_) {
    tier4_external_api_msgs::msg::ResponseStatus response_status;
    response_status.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
    response_status.message = "OK";
    pub_sound_response_initialpose_->publish(response_status);
    is_playing_sound_initialpose_ = false;
  }
}

// callbackAwapiVehicleState() removed - replaced by callbackAdapiVehicleStatus()

void AdSoundManager::callbackSoundRequestInitialpose(const sound_msgs::msg::SoundRequest::ConstSharedPtr msg)
{
  std::string file_path = sound_filename_alert_imu_initialize_;
  bool cut_in = false;
  playOneshotVoice(file_path, cut_in);
  is_playing_sound_initialpose_ = true;
}

const audio_driver_msgs::msg::SoundDriverCtrl AdSoundManager::initAudioCmd(
  const int cmd_type, const float volume,
  const std::string file_path, const bool is_loop,
  const float loop_delay, const float start_delay) const
{
  audio_driver_msgs::msg::SoundDriverCtrl sound_msg;
  sound_msg.cmd_type = cmd_type;
  sound_msg.volume = volume;
  sound_msg.file_path = file_path;
  sound_msg.is_loop = is_loop;
  sound_msg.loop_delay = loop_delay;
  sound_msg.start_delay = start_delay;
  sound_msg.stamp = this->now();
  return sound_msg;
}

void AdSoundManager::playOneshotVoice(const std::string file_path, const bool cut_in)
{
  if (cut_in)
  {
    pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
  }
  const bool is_loop = false;
  auto cmd = initAudioCmd(sdc_msg_.CMD_PLAY, VOLUME_VOICE_ALARM, file_path, is_loop);
  pub_voice_cmd_->publish(cmd);
  pre_sound_filename_ = file_path;
}

void AdSoundManager::playLoopVoice(const std::string file_path,
  const bool cut_in, const bool is_long_delay)
{
  if (cut_in)
  {
    pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
  }
  const bool is_loop = true;
  const double loop_delay = (is_long_delay) ? (LOOP_DELAY_VOICE_ALARM_LONG) : (LOOP_DELAY_VOICE_ALARM);
  const double start_delay = (cut_in) ? (START_DELAY_CHANGE) : (START_DELAY_NONE);
  auto cmd = initAudioCmd(sdc_msg_.CMD_PLAY, VOLUME_VOICE_ALARM, file_path, is_loop, loop_delay, start_delay);
  pub_voice_cmd_->publish(cmd);
  pre_sound_filename_ = file_path;
}

void AdSoundManager::playLoopNoBGM(const std::string file_path)
{
  pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
  const bool is_loop = true;
  auto cmd = initAudioCmd(sdc_msg_.CMD_PLAY, VOLUME_ZERO_BGM, file_path, is_loop);
  pub_bgm_cmd_->publish(cmd);
}

void AdSoundManager::playLoopBGM(const std::string file_path)
{
  auto cmd = initAudioCmd(sdc_msg_.CMD_PLAY, VOLUME_HIGH_BGM, file_path);
  pub_bgm_cmd_->publish(cmd);
}

AdSoundManager::PreSoundType AdSoundManager::checkPreSoundType(void)
{
  if ( (pre_sound_filename_ == sound_filename_left_) ||
    (pre_sound_filename_ == sound_filename_right_) )
  {
    return PreSoundType::TURN_LEFTRIGHT_SOUND;
  } else if ( (pre_sound_filename_ == sound_filename_obstacle_) ||
    (pre_sound_filename_ == sound_filename_leave_) ||
    (pre_sound_filename_ == sound_filename_avoid_) )
  {
    return PreSoundType::STOP_REASON_SOUND;
  } else if (pre_sound_filename_ == "") {
    return PreSoundType::SOUND_NONE;
  } else {
    return PreSoundType::NORMAL_SOUND;
  }
}

void AdSoundManager::changeSoundState(
  const uint16_t service_layer_state,
  const uint8_t control_layer_state,
  bool force)
{
  // DEBUG: Log changeSoundState entry
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] changeSoundState() called: service=%u, control=%u, force=%d",
    service_layer_state, control_layer_state, force);

  prev_service_layer_state_ = cur_service_layer_state_;
  prev_control_layer_state_ = cur_control_layer_state_;
  cur_service_layer_state_ = service_layer_state;
  cur_control_layer_state_ = control_layer_state;

  // When state is STATE_CHECK_NODE_ALIVE, STATE_WAITING_CALL_PERMISSION,
  //   STATE_INFORM_ENGAGE, or STATE_INFORM_RESTART,
  //   play the same audio regardless of manual and auto.
  const auto is_ignore_control_layer_state =
    (cur_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE) ||
    (cur_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION) ||
    (cur_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE) ||
    (cur_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART);

  if (cur_service_layer_state_ == prev_service_layer_state_) {
    // The variable "force" is set to true only when turn signal is changed.
    if (force == true) {
      if ( (cur_service_layer_state_ !=
        autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_STOP_LINE) &&
        (cur_service_layer_state_ !=
        autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_OBSTACLE) &&
        (cur_service_layer_state_ !=
        autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION) )
      {
        // Turn signal information is used only when state is STATE_RUNNING_TOWARD_STOP_LINE,
        //   STATE_RUNNING_TOWARD_OBSTACLE or STATE_STOP_DUETO_TRAFFIC_CONDITION. If the state is
        //   other than that, return immediately.
        RCLCPP_DEBUG(this->get_logger(),
          "[DEBUG] changeSoundState() early return: force=true but state not applicable");
        return;
      }
    } else {
      // If state has not changed, or if the same audio is played manual and auto,
      //  return immediately.
      if ((cur_control_layer_state_ == prev_control_layer_state_) ||
        is_ignore_control_layer_state)
      {
        RCLCPP_DEBUG(this->get_logger(),
          "[DEBUG] changeSoundState() early return: state unchanged (service=%u, control=%u, is_ignore=%d)",
          cur_service_layer_state_, cur_control_layer_state_, is_ignore_control_layer_state);
        return;
      }
    }
  }

  // DEBUG: Log state change will be processed
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] changeSoundState() processing state change: prev_service=%u -> cur_service=%u, control=%u",
    prev_service_layer_state_, cur_service_layer_state_, cur_control_layer_state_);

  if (!is_ignore_control_layer_state &&
    (cur_control_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::MANUAL))
  {
    RCLCPP_INFO(this->get_logger(),
      "[DEBUG] changeSoundState() MANUAL mode, stopping voice");
    continuity_state_ = false;
    playLoopNoBGM(sound_filename_bgm_);
    pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
    pre_sound_filename_ = "";
    return;
  }
  // Previously played audio file information affects
  //   the next audio playback parameters.
  auto pre_sound_type = checkPreSoundType();
  const bool is_cut_in_voice =
    (pre_sound_type == PreSoundType::TURN_LEFTRIGHT_SOUND) ||
    (pre_sound_type == PreSoundType::STOP_REASON_SOUND);
  // DEBUG: Log switch state
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] changeSoundState() switch: cur_service_layer_state_=%u", cur_service_layer_state_);

  switch (cur_service_layer_state_) {
    case autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE:
      RCLCPP_WARN(this->get_logger(),
        "[DEBUG] === WAKEUP SOUND === STATE_CHECK_NODE_ALIVE, playing: %s",
        sound_filename_wakeup_.c_str());
      continuity_state_ = false;
      one_play_state_ = cur_service_layer_state_;
      playOneshotVoice(sound_filename_wakeup_);
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE:
      pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
      continuity_state_ = false;
      playLoopVoice(sound_filename_obstacle_, is_cut_in_voice);
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY:
      pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
      continuity_state_ = false;
      playLoopVoice(sound_filename_leave_, is_cut_in_voice);
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION:
      continuity_state_ = false;
      playLoopNoBGM(sound_filename_bgm_);
      pre_sound_filename_ = "";
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION:
      playLoopNoBGM(sound_filename_bgm_);
      continuity_state_ = false;
      playLoopVoice(sound_filename_call_, false, true);
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART:
      RCLCPP_WARN(this->get_logger(),
        "[DEBUG] === START SOUND === STATE_INFORM_ENGAGE/RESTART, playing: %s, one_play_state_=%u",
        sound_filename_start_.c_str(), cur_service_layer_state_);
      pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
      continuity_state_ = false;
      one_play_state_ = cur_service_layer_state_;
      playOneshotVoice(sound_filename_start_, (pre_sound_type != PreSoundType::SOUND_NONE));
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_OBSTACLE_AVOIDANCE:
      pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
      continuity_state_ = false;
      playLoopVoice(sound_filename_avoid_, is_cut_in_voice);
      break;

    case autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_LEFT:
      if ( (continuity_state_ == true) && (turn_state_ == LEFT) ) {
        // Loop playback has already been instructed.
        break;
      }
      turn_state_ = LEFT;
      continuity_state_ = true;
      pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
      playLoopVoice(sound_filename_left_, is_cut_in_voice);
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_RIGHT:
      if ( (continuity_state_ == true) && (turn_state_ == RIGHT) ) {
        // Loop playback has already been instructed.
        break;
      }
      turn_state_ = RIGHT;
      continuity_state_ = true;
      pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
      playLoopVoice(sound_filename_right_, is_cut_in_voice);
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING:
      turn_state_ = NORMAL;
      continuity_state_ = false;
      pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
      pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_HIGH_BGM));
      pre_sound_filename_ = "";
      break;

    case autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_STOP_LINE:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_OBSTACLE:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION:
      if (continuity_state_ == false) {
        if (turn_signal_ == tier4_vehicle_msgs::msg::TurnSignal::LEFT) {
          turn_state_ = LEFT;
          continuity_state_ = true;
          pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
          playLoopVoice(sound_filename_left_, is_cut_in_voice);
          break;
        }
        else if (turn_signal_ == tier4_vehicle_msgs::msg::TurnSignal::RIGHT) {
          turn_state_ = RIGHT;
          continuity_state_ = true;
          pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
          playLoopVoice(sound_filename_right_, is_cut_in_voice);
          break;
        }
        else
        {
          // Continues the current playback for a certain period of time.
          rclcpp::Rate(1 / CURRENT_PLAY_DURATION).sleep();
          pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
          pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_HIGH_BGM));
          pre_sound_filename_ = "";
        }
        break;
      }
      // continuity_state_ == true
      if (turn_state_ == LEFT) {
        if (turn_signal_ == tier4_vehicle_msgs::msg::TurnSignal::RIGHT) {
          turn_state_ = RIGHT;
          pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
          playLoopVoice(sound_filename_right_, is_cut_in_voice);
          break;
        } else if (turn_signal_ != tier4_vehicle_msgs::msg::TurnSignal::LEFT) {
          turn_state_ = NORMAL;
          continuity_state_ = false;
          pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
          pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_HIGH_BGM));
          pre_sound_filename_ = "";
          break;
        }
      }
      if (turn_state_ == RIGHT) {
        if (turn_signal_ == tier4_vehicle_msgs::msg::TurnSignal::LEFT) {
          turn_state_ = LEFT;
          pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
          playLoopVoice(sound_filename_left_, is_cut_in_voice);
          break;
        } else if (turn_signal_ != tier4_vehicle_msgs::msg::TurnSignal::RIGHT) {
          turn_state_ = NORMAL;
          continuity_state_ = false;
          pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
          pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_HIGH_BGM));
          pre_sound_filename_ = "";
          break;
        }
      }
      break;

    case autoware_state_machine_msgs::msg::StateMachine::STATE_INSTRUCT_ENGAGE:
      continuity_state_ = false;
      pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
      playLoopBGM(sound_filename_bgm_);
      pre_sound_filename_ = "";
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL:
      RCLCPP_WARN(this->get_logger(),
        "[DEBUG] === ARRIVAL SOUND === STATE_ARRIVED_GOAL, playing: %s",
        sound_filename_arrival_.c_str());
      pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
      continuity_state_ = false;
      one_play_state_ = cur_service_layer_state_;
      playOneshotVoice(sound_filename_arrival_, is_cut_in_voice);
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_CLOSE:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE:
      playLoopNoBGM(sound_filename_bgm_);
      continuity_state_ = false;
      pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
      pre_sound_filename_ = "";
      break;
    default:
      break;
  }
}

// ============================================================
// Callback functions - New ADAPI v1
// ============================================================

void AdSoundManager::callbackMotionState(
  const autoware_adapi_v1_msgs::msg::MotionState::ConstSharedPtr msg)
{
  prev_motion_state_ = motion_state_;
  motion_state_ = *msg;

  // DEBUG: Log motion state
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] callbackMotionState: state=%u (0:UNKNOWN, 1:STOPPED, 2:STARTING, 3:MOVING), prev=%u",
    motion_state_.state, prev_motion_state_.state);

  // Call updateAutowareStateFromTopics() when state changes
  if (prev_motion_state_.state != motion_state_.state) {
    RCLCPP_INFO(this->get_logger(),
      "[DEBUG] MotionState CHANGED: %u -> %u, calling updateAutowareStateFromTopics()",
      prev_motion_state_.state, motion_state_.state);
    updateAutowareStateFromTopics();
  }
}

void AdSoundManager::callbackAdapiVehicleStatus(
  const autoware_adapi_v1_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  prev_adapi_vehicle_status_ = adapi_vehicle_status_;
  adapi_vehicle_status_ = *msg;

  // Call updateAutowareStateFromTopics() when turn_indicators changes
  if (prev_adapi_vehicle_status_.turn_indicators.status != adapi_vehicle_status_.turn_indicators.status) {
    updateAutowareStateFromTopics();
  }
}

void AdSoundManager::callbackRouteState(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  prev_route_state_ = route_state_;
  route_state_ = *msg;

  // DEBUG: Log route state
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] callbackRouteState: state=%u (0:UNKNOWN, 1:UNSET, 2:SET, 3:ARRIVED, 4:CHANGING), prev=%u",
    route_state_.state, prev_route_state_.state);

  // Call updateAutowareStateFromTopics() when state changes
  if (prev_route_state_.state != route_state_.state) {
    RCLCPP_INFO(this->get_logger(),
      "[DEBUG] RouteState CHANGED: %u -> %u, calling updateAutowareStateFromTopics()",
      prev_route_state_.state, route_state_.state);
    updateAutowareStateFromTopics();
  }
}

void AdSoundManager::callbackLocalizationState(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg)
{
  using LocalizationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using StateMachine = autoware_state_machine_msgs::msg::StateMachine;

  prev_localization_state_ = localization_state_;
  localization_state_ = *msg;

  // DEBUG: Log localization state
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] callbackLocalizationState: state=%u (0:UNKNOWN, 1:UNINITIALIZED, 2:INITIALIZING, 3:INITIALIZED), prev=%u",
    localization_state_.state, prev_localization_state_.state);

  // Call updateAutowareStateFromTopics() when state changes
  if (prev_localization_state_.state != localization_state_.state) {
    RCLCPP_INFO(this->get_logger(),
      "[DEBUG] LocalizationState CHANGED: %u -> %u",
      prev_localization_state_.state, localization_state_.state);

    // When localization transitions TO INITIALIZED, play wakeup sound (STATE_CHECK_NODE_ALIVE)
    if (prev_localization_state_.state != LocalizationState::INITIALIZED &&
        localization_state_.state == LocalizationState::INITIALIZED)
    {
      RCLCPP_WARN(this->get_logger(),
        "[DEBUG] Localization transitioned to INITIALIZED -> triggering STATE_CHECK_NODE_ALIVE (wakeup)");
      changeSoundState(StateMachine::STATE_CHECK_NODE_ALIVE, cur_control_layer_state_, false);
    }
    else {
      updateAutowareStateFromTopics();
    }
  }
}

void AdSoundManager::callbackOperationModeState(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  prev_operation_mode_state_ = operation_mode_state_;
  operation_mode_state_ = *msg;

  // Call updateAutowareStateFromTopics() when mode or control_enabled changes
  if (prev_operation_mode_state_.mode != operation_mode_state_.mode ||
      prev_operation_mode_state_.is_autoware_control_enabled != operation_mode_state_.is_autoware_control_enabled)
  {
    updateAutowareStateFromTopics();
  }
}

// ============================================================
// Callback functions - go_interface
// ============================================================

void AdSoundManager::callbackGoInterfaceVehicleStatus(
  const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  prev_go_interface_vehicle_status_ = go_interface_vehicle_status_;
  go_interface_vehicle_status_ = *msg;

  // Call updateAutowareStateFromTopics() when voice_flg or lock_flg changes
  if (prev_go_interface_vehicle_status_.voice_flg != go_interface_vehicle_status_.voice_flg ||
      prev_go_interface_vehicle_status_.lock_flg != go_interface_vehicle_status_.lock_flg)
  {
    updateAutowareStateFromTopics();
  }
}

// ============================================================
// Callback functions - Legacy (TODO: Replace with planning_factors)
// ============================================================

void AdSoundManager::callbackAwapiAutowareStatus(
  const tier4_api_msgs::msg::AwapiAutowareStatus::ConstSharedPtr msg)
{
  awapi_autoware_status_ = *msg;

  // TODO: Replace with /api/external/get/planning_factors for stop reason detection
}

// ============================================================
// State conversion function
// ============================================================

void AdSoundManager::updateAutowareStateFromTopics(void)
{
  // This function converts new ADAPI v1 states to legacy service_layer_state and control_layer_state
  // and calls changeSoundState()

  using StateMachine = autoware_state_machine_msgs::msg::StateMachine;
  using MotionState = autoware_adapi_v1_msgs::msg::MotionState;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using LocalizationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using OperationMode = autoware_adapi_v1_msgs::msg::OperationModeState;
  using TurnIndicators = autoware_adapi_v1_msgs::msg::TurnIndicators;

  // ワンショット音声再生中は状態遷移をスキップ
  // STATE_CHECK_NODE_ALIVE (wakeup), STATE_ARRIVED_GOAL (arrival),
  // STATE_INFORM_ENGAGE, STATE_INFORM_RESTART の音声再生中は状態を変えない
  if (one_play_state_ == StateMachine::STATE_ARRIVED_GOAL ||
      one_play_state_ == StateMachine::STATE_CHECK_NODE_ALIVE ||
      one_play_state_ == StateMachine::STATE_INFORM_ENGAGE ||
      one_play_state_ == StateMachine::STATE_INFORM_RESTART)
  {
    RCLCPP_DEBUG(this->get_logger(),
      "[DEBUG] One-shot sound playing (state=%u), skipping state update", one_play_state_);
    return;
  }

  // DEBUG: Log current ADAPI states
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] updateAutowareStateFromTopics() called with:\n"
    "  localization_state=%u, route_state=%u, motion_state=%u\n"
    "  operation_mode=%u, is_autoware_control_enabled=%d",
    localization_state_.state, route_state_.state, motion_state_.state,
    operation_mode_state_.mode, operation_mode_state_.is_autoware_control_enabled);

  // ============================================================
  // Derive control_layer_state
  // ============================================================
  uint8_t control_layer_state = StateMachine::MANUAL;
  std::string control_reason = "default MANUAL";
  if (operation_mode_state_.mode == OperationMode::AUTONOMOUS &&
      operation_mode_state_.is_autoware_control_enabled)
  {
    control_layer_state = StateMachine::AUTO;
    control_reason = "AUTONOMOUS + control_enabled";
  } else if (operation_mode_state_.mode != OperationMode::AUTONOMOUS) {
    control_reason = "mode is not AUTONOMOUS (mode=" + std::to_string(operation_mode_state_.mode) + ")";
  } else if (!operation_mode_state_.is_autoware_control_enabled) {
    control_reason = "AUTONOMOUS but control_enabled=false";
  }
  
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] control_layer_state=%u, reason=%s",
    control_layer_state, control_reason.c_str());

  // ============================================================
  // Derive service_layer_state (priority order)
  // ============================================================
  uint16_t service_layer_state = StateMachine::STATE_UNDEFINED;
  std::string state_reason = "DEFAULT";

  // TODO: Emergency stop detection
  // if (emergency_holding) {
  //   service_layer_state = StateMachine::STATE_EMERGENCY_STOP;
  // }

  // 1. Localization not initialized
  // Note: STATE_CHECK_NODE_ALIVE (wakeup) is handled in callbackLocalizationState()
  //       when localization transitions to INITIALIZED
  if (localization_state_.state != LocalizationState::INITIALIZED) {
    service_layer_state = StateMachine::STATE_DURING_WAKEUP;
    state_reason = "localization not INITIALIZED";
  }
  // 2. Arrived at goal
  else if (route_state_.state == RouteState::ARRIVED) {
    service_layer_state = StateMachine::STATE_ARRIVED_GOAL;
    state_reason = "route ARRIVED";
  }
  // 3. Route not set or changing
  else if (route_state_.state == RouteState::UNSET ||
           route_state_.state == RouteState::CHANGING)
  {
    service_layer_state = StateMachine::STATE_DURING_RECEIVE_ROUTE;
    state_reason = "route UNSET or CHANGING";
  }
  // 4. Starting (departure sound)
  else if (motion_state_.state == MotionState::STARTING) {
    // No distinction between initial departure and restart
    service_layer_state = StateMachine::STATE_INFORM_ENGAGE;
    state_reason = "motion STARTING";
  }
  // 5. Waiting for call permission (external system integration)
  else if (go_interface_vehicle_status_.voice_flg &&
           go_interface_vehicle_status_.lock_flg)
  {
    service_layer_state = StateMachine::STATE_WAITING_CALL_PERMISSION;
    state_reason = "voice_flg && lock_flg";
  }
  // 6. Waiting for engage instruction
  else if (route_state_.state == RouteState::SET &&
           control_layer_state == StateMachine::MANUAL)
  {
    service_layer_state = StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;
    state_reason = "route SET && MANUAL mode";
  }
  // 7. Moving
  else if (motion_state_.state == MotionState::MOVING) {
    // Check turn indicators
    if (adapi_vehicle_status_.turn_indicators.status == TurnIndicators::LEFT) {
      service_layer_state = StateMachine::STATE_TURNING_LEFT;
      state_reason = "motion MOVING + turn LEFT";
    } else if (adapi_vehicle_status_.turn_indicators.status == TurnIndicators::RIGHT) {
      service_layer_state = StateMachine::STATE_TURNING_RIGHT;
      state_reason = "motion MOVING + turn RIGHT";
    } else {
      service_layer_state = StateMachine::STATE_RUNNING;
      state_reason = "motion MOVING";
    }
  }
  // 8. Stopped
  else if (motion_state_.state == MotionState::STOPPED) {
    // TODO: Replace with /api/external/get/planning_factors for stop reason detection
    // For now, use stop_reason from awapi_autoware_status_ if available
    // Default to STATE_STOP_DUETO_TRAFFIC_CONDITION
    service_layer_state = StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION;
    state_reason = "motion STOPPED";
  }
  // 9. Default
  else {
    service_layer_state = StateMachine::STATE_UNDEFINED;
    state_reason = "no match";
  }

  // DEBUG: Log derived states
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] Derived states: service_layer_state=%u, control_layer_state=%u, reason=%s\n"
    "  (prev_service=%u, prev_control=%u)",
    service_layer_state, control_layer_state, state_reason.c_str(),
    cur_service_layer_state_, cur_control_layer_state_);

  // Call changeSoundState with derived states
  changeSoundState(service_layer_state, control_layer_state, false);
}

}  // namespace ad_sound_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ad_sound_manager::AdSoundManager)
