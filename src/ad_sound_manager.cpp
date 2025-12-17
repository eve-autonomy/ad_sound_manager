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
  // Parameter
  double stop_dist_to_prohibit_engage = this->declare_parameter<double>("stop_dist_to_prohibit_engage", 0.30);
  // Add a value of 0.05 to `stop_dist_to_prohibit_engage`.
  dist_to_stop_pose_min_th_ = stop_dist_to_prohibit_engage + 0.05;

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

  // autoware_state
  sub_initilization_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
    "/api/localization/initialization_state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onAutowareInitializationMessage, this, std::placeholders::_1)
  );

  // routing wait state
  sub_routing_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onRoutingStateMessage, this, std::placeholders::_1)
  );
  sub_routing_route_ = this->create_subscription<autoware_adapi_v1_msgs::msg::Route>(
    "/api/routing/route",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onRoutingRouteMessage, this, std::placeholders::_1)
  );

  // daignostics struct for EM Holding
  sub_daignostics_struct_ = this->create_subscription<autoware_adapi_v1_msgs::msg::DiagGraphStruct>(
    "/api/system/diagnostics/struct",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onDaignosticsStructMessage, this, std::placeholders::_1)
  );
  sub_daignostics_status_ = this->create_subscription<autoware_adapi_v1_msgs::msg::DiagGraphStatus>(
    "/api/system/diagnostics/status",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onDaignosticsStateMessage, this, std::placeholders::_1)
  );

  // OperationModeState
  sub_operation_mode_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onOperationModeStateMessage, this, std::placeholders::_1)
  );

  // vehicle_status
  sub_calls_vehicle_state_ = this->create_subscription<go_interface_msgs::msg::VehicleStatus>(
    "api_vehicle_status",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onVehicleStateMessage, this, std::placeholders::_1)
  );

  // delivery reservation state
  sub_delivery_reservation_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateLock>(
    "/go_interface/lock_state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onDeliveryReservationMessage, this, std::placeholders::_1)
  );

  // engage process state
  sub_engage_process_state_ = this->create_subscription<eve_cmd_gate_msgs::msg::EngageRequestState>(
    "/eve_cmd_gate/engage_request_state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onEngageProcessMessage, this, std::placeholders::_1)
  );

  // velocity
  sub_vehicle_kinematics_ = this->create_subscription<autoware_adapi_v1_msgs::msg::VehicleKinematics>(
    "/api/vehicle/kinematics",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onVehicleKinematicsMessage, this, std::placeholders::_1)
  );

  // turn_signal
  sub_vehicle_status_ = this->create_subscription<autoware_adapi_v1_msgs::msg::VehicleStatus>(
    "/api/vehicle/status",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onVehicleStatusMessage, this, std::placeholders::_1)
  );

  // stop reasons
  sub_planning_factors_ = this->create_subscription<tier4_api_msgs::msg::AwapiAutowareStatus>(
        "/awapi/autoware/get/status", rclcpp::QoS{1},
    std::bind(&AdSoundManager::onPlanningFactorsMessage, this, std::placeholders::_1)
  );
/*
  sub_planning_factors_ = this->create_subscription<tier4_planning_msgs::msg::PlanningFactorArray>(
    "/api/external/get/planning_factors",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::onPlanningFactorsMessage, this, std::placeholders::_1)
  );
*/

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

  em_holding_indices_ = std::nullopt;
  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  initilization_state_ = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNKNOWN;
  routing_state_ = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
  delivery_reservation_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
  em_holding_ = false;
  operation_state_.is_autoware_control_enabled = false;
  operation_state_.is_in_transition = false;
  operation_state_.is_stop_mode_available = false;
  operation_state_.is_autonomous_mode_available = false;
  operation_state_.is_local_mode_available = false;
  operation_state_.is_remote_mode_available = false;
  is_obstacle_stop_ = false;
  is_detection_area_ = false;
  is_crosswalk_ = false;
  is_surround_obstacle_check_ = false;
  is_stop_reason_ = false;
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

  one_play_done_flag_ = true;
  changeState();

  one_play_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  one_play_done_flag_ = false;
}

//void AdSoundManager::callbackAutowareStateMachine(
//  const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg)
//{
//  RCLCPP_INFO_THROTTLE(
//    this->get_logger(),
//    *this->get_clock(), 1.0,
//    "[AdSoundManager::callback]service_layer_state: %u, control_layer_state: %u",
//    msg->service_layer_state,
//    msg->control_layer_state);
//
//  changeSoundState(msg->service_layer_state, msg->control_layer_state, false);
//}

void AdSoundManager::callbackVoiceRes(
  const audio_driver_msgs::msg::SoundDriverRes::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::callbackVoiceRes]callbackVoiceRes");

  // Done information is returned only after one-time playback.
  if ( (one_play_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE) ||
    (one_play_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL) ||
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

//void AdSoundManager::callbackAwapiVehicleState(
//  const tier4_api_msgs::msg::AwapiVehicleStatus::ConstSharedPtr msg)
//{
//  if (turn_signal_ != msg->turn_signal) {
//    turn_signal_ = msg->turn_signal;
//    changeSoundState(cur_service_layer_state_, cur_control_layer_state_, true);
//  }
//}

void AdSoundManager::callbackSoundRequestInitialpose(const sound_msgs::msg::SoundRequest::ConstSharedPtr msg)
{
  std::string file_path = sound_filename_alert_imu_initialize_;
  bool cut_in = false;
  playOneshotVoice(file_path, cut_in);
  is_playing_sound_initialpose_ = true;
}

void AdSoundManager::onAutowareInitializationMessage(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::onAutowareInitializationMessage]autoware_state: %u",
    msg->state);

  initilization_state_ = msg->state;

  changeState();
}

void AdSoundManager::onRoutingStateMessage(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::onRoutingStateMessage]routing_state: %u",
    msg->state);

  routing_state_ = msg->state;

  changeState();
}

void AdSoundManager::onRoutingRouteMessage(
  const autoware_adapi_v1_msgs::msg::Route::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::onRoutingRouteMessage]data.size: %lu",
    msg->data.size());

  routing_data_size_ = msg->data.size();

  changeState();
}

void AdSoundManager::onDaignosticsStructMessage(
  const autoware_adapi_v1_msgs::msg::DiagGraphStruct::ConstSharedPtr msg)
{
  auto nodes = msg->nodes;

  for (uint16_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].path == "/autoware/modes/autonomous") {
      em_holding_indices_ = i;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1.0,
        "[AdSoundManager::onDaignosticsStructMessage]daignostics_graph /autoware/modes/autonomous index: %u", i);
      break;
    }
  }
}

void AdSoundManager::onDaignosticsStateMessage(
  const autoware_adapi_v1_msgs::msg::DiagGraphStatus::ConstSharedPtr msg)
{
  auto nodes = msg->nodes;
  if (em_holding_indices_ != std::nullopt) {
    // TODO:Ph3にて、levelをlatch_levelに変更
    if (nodes[em_holding_indices_.value()].level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      em_holding_ = true;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1.0,
        "[AdSoundManager::onDaignosticsStateMessage]/autoware/modes/autonomous latch_level: %u",
        nodes[em_holding_indices_.value()].level);// TODO:Ph3にて、levelをlatch_levelに変更
    } else {
      em_holding_ = false;
    }

    changeState();
  }
}

void AdSoundManager::onOperationModeStateMessage(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  operation_state_ = *msg;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::onOperationModeStateMessage]operation mode: %u",
      msg->mode);

  changeState();
}

void AdSoundManager::onVehicleStateMessage(
  const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  flag_calls_vehicle_voice_ = msg->voice_flg;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::onOperationModeStateMessage]vheicle voice: %u",
      msg->voice_flg);

  changeState();
}

void AdSoundManager::onDeliveryReservationMessage(
  const autoware_state_machine_msgs::msg::StateLock::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::onDeliveryReservationMessage]"
    "StateLock: %u",
    msg->state);

  delivery_reservation_state_ = msg->state;
  changeState();
}

void AdSoundManager::onEngageProcessMessage(
  const eve_cmd_gate_msgs::msg::EngageRequestState::ConstSharedPtr msg)
{
  is_engage_requesting_ = msg->is_engage_requesting;
  is_engage_accepted_ = msg->is_engage_accepted;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::onEngageProcessMessage]"
    "request: %u, accept: %u",
    is_engage_requesting_, is_engage_accepted_);

  changeState();
}

void AdSoundManager::onVehicleKinematicsMessage(
  const autoware_adapi_v1_msgs::msg::VehicleKinematics::ConstSharedPtr msg)
{
  velocity_ = msg->twist.twist.twist.linear.x;
  vehicle_pose_ = msg->pose.pose.pose;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::onVehicleKinematicsMessage]"
    "velocity: %f, pose: [%f,%f,%f]",
    msg->twist.twist.twist.linear.x,
    msg->pose.pose.pose.position.x,msg->pose.pose.pose.position.y,msg->pose.pose.pose.position.z);

  changeState();
}

void AdSoundManager::onVehicleStatusMessage(
  const autoware_adapi_v1_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  auto pre_turn_signal_ = turn_signal_;
  turn_signal_ = msg->turn_indicators.status;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::onVehicleStatusMessage]"
    "turn_signal: %u",
    msg->turn_indicators.status);

  if (pre_turn_signal_ != turn_signal_) {
    changeSoundState(service_layer_state_, control_layer_state_, true);
  }
}

void AdSoundManager::onPlanningFactorsMessage(
    const tier4_api_msgs::msg::AwapiAutowareStatus::ConstSharedPtr msg)
  //  const tier4_planning_msgs::msg::PlanningFactorArray::ConstSharedPtr msg)
{
  bool is_other_factor = false;
  is_obstacle_stop_ = false;
  is_detection_area_ = false;
  is_crosswalk_ = false;
  is_surround_obstacle_check_ = false;
/*
  for (const auto & factor : msg->factors) {
    if (factor.behavior_type == tier4_planning_msgs::msg::PlanningFactor::STOP) {
      if (factor.behavior_name == tier4_planning_msgs::msg::PlanningFactor::ROUTE_OBSTACLE) {
        is_obstacle_stop_ = true;
        dist_to_stop_pose_ = factor.control_points.distance;
      }
      else if (factor.behavior_name == tier4_planning_msgs::msg::PlanningFactor::USER_DEFINED_DETECTION_AREA) {
        is_detection_area_ = true;
        dist_to_stop_pose_ = factor.control_points.distance;
      }
      else if (factor.behavior_name == tier4_planning_msgs::msg::PlanningFactor::CROSSWALK) {
        is_crosswalk_ = true;
        dist_to_stop_pose_ = factor.control_points.distance;
      }
      else if (factor.behavior_name == tier4_planning_msgs::msg::PlanningFactor::SURROUNDING_OBSTACLE) {
        is_surround_obstacle_check_ = true;
        dist_to_stop_pose_ = factor.control_points.distance;
      }
      else {
        is_other_factor = true;
      }
    }
  }
*/
  is_stop_reason_ = is_obstacle_stop_ | is_detection_area_ | is_crosswalk_ | is_surround_obstacle_check_ | is_other_factor;

  /*
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::onPlanningFactorsMessage]"
    "size: %lu, is_stop_reason: %u, is_obstacle_stop: %u, is_detection_area_: %u, is_crosswalk_: %u, is_surround_obstacle_check_: %u",
    msg->factors.size(), is_stop_reason_, is_obstacle_stop_, is_detection_area_, is_crosswalk_, is_surround_obstacle_check_);
  */

  changeState();
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
        return;
      }
    } else {
      // If state has not changed, or if the same audio is played manual and auto,
      //  return immediately.
      if ((cur_control_layer_state_ == prev_control_layer_state_) ||
        is_ignore_control_layer_state)
      {
        return;
      }
    }
  }

  if (!is_ignore_control_layer_state &&
    (cur_control_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::MANUAL))
  {
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
  switch (cur_service_layer_state_) {
    case autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE:
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

bool AdSoundManager::isAutowareStateOfInitializingVehicle(void)
{
  if (initilization_state_ == autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) {
    return false;
  }

  return true;
}

bool AdSoundManager::isAutowareStateOfWaitingForRoute(void)
{
  if ((routing_state_ != autoware_adapi_v1_msgs::msg::RouteState::UNSET) &&
      (routing_state_ != autoware_adapi_v1_msgs::msg::RouteState::CHANGING)) {
    return false;
  }
  if ((initilization_state_ != autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) ||
      (routing_data_size_ != 0)) {
    return false;
  }

  return true;
}

bool AdSoundManager::isAutowareStateOfPlanning(void)
{
  if ((initilization_state_ != autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) ||
      (routing_state_ != autoware_adapi_v1_msgs::msg::RouteState::SET) ||
      (routing_data_size_ != 0) ||
      (operation_state_.mode != autoware_adapi_v1_msgs::msg::OperationModeState::STOP) ||
      (operation_state_.is_autoware_control_enabled == true) ||
      (operation_state_.is_autonomous_mode_available == true)) {
    return false;
  }

  return true;
}

bool AdSoundManager::isAutowareStateOfWaitingForEngage(void)
{
  if ((initilization_state_ != autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) ||
      (routing_state_ != autoware_adapi_v1_msgs::msg::RouteState::SET) ||
      (routing_data_size_ == 0) ||
      (operation_state_.mode != autoware_adapi_v1_msgs::msg::OperationModeState::STOP) ||
      (operation_state_.is_autoware_control_enabled == true) ||
      (operation_state_.is_autonomous_mode_available != true)) {
    return false;
  }

  return true;
}

bool AdSoundManager::isAutowareStateOfDriving(void)
{
  if ((initilization_state_ != autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) ||
      (routing_state_ != autoware_adapi_v1_msgs::msg::RouteState::SET) ||
      (routing_data_size_ == 0) ||
      (operation_state_.mode != autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS) ||
      (operation_state_.is_autoware_control_enabled != true) ||
      (operation_state_.is_in_transition == true)) {
    return false;
  }

  return true;
}

bool AdSoundManager::isAutowareStateOfArrivedGoal(void)
{
  if ((initilization_state_ != autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED) ||
      (routing_state_ != autoware_adapi_v1_msgs::msg::RouteState::ARRIVED)) {
    return false;
  }

  return true;
}

bool AdSoundManager::changeCheckNodeAlive(void)
{
  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE;
  return true;
}

bool AdSoundManager::changeStateDuringWakeUp(void)
{
  if ((is_engage_requesting_ == true) ||
      (one_play_state_ != service_layer_state_) ||
      (one_play_state_ != autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE) ||
      (one_play_done_flag_ != true)) {
    return false;
  } else {
    one_play_done_flag_ = false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP;
  return true;
}

bool AdSoundManager::changeStateDuringReceiveRoute(void)
{
  if ((is_engage_requesting_ == true) ||
      (one_play_state_ != service_layer_state_) ||
      (one_play_state_ != autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE) ||
      (one_play_done_flag_ != true)) {
    if (service_layer_state_ != autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP) {
      return false;
    }
  } else {
    one_play_done_flag_ = false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
  return true;
}

bool AdSoundManager::changeStateWaitingEngageInstruction(void)
{
  if ((is_engage_requesting_ == true) ||
      (one_play_state_ != service_layer_state_) ||
      (one_play_state_ != autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE) ||
      (one_play_done_flag_ != true)) {
    if (service_layer_state_ != autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_WAKEUP) {
      return false;
    }
  } else {
    one_play_done_flag_ = false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;
  return true;
}

bool AdSoundManager::changeStateWaitingCallPermission(void)
{
  if ((is_engage_requesting_ == true) ||
      (flag_calls_vehicle_voice_ != true) ||
      (delivery_reservation_state_ != autoware_state_machine_msgs::msg::StateLock::STATE_ON)) {
      return false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION;
  return true;
}

bool AdSoundManager::changeStateInformEngage(void)
{
  if ((is_engage_requesting_ != true) ||
      (flag_calls_vehicle_voice_ == true) ||
      (delivery_reservation_state_ != autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION)) {
    return false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE;
  return true;
}

bool AdSoundManager::changeStateInstructEngage(void)
{
  if ((is_engage_requesting_ == true) ||
      (one_play_state_ != service_layer_state_) ||
      (one_play_state_ != autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE) ||
      (one_play_done_flag_ != true)) {
    return false;
  } else {
    one_play_done_flag_ = false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_INSTRUCT_ENGAGE;
  return true;
}

bool AdSoundManager::changeStateRunning(void)
{
  if ((is_engage_requesting_ == true) ||
      (is_stop_reason_ == true) ||
      (turn_signal_ != autoware_adapi_v1_msgs::msg::TurnIndicators::DISABLE)) {
    return false;
  }

  if ((one_play_state_ == service_layer_state_) &&
      (one_play_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART)) {
    if (one_play_done_flag_ != true) {
      return false;
    } else {
      one_play_done_flag_ = false;
    }
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING;
  return true;
}

bool AdSoundManager::changeStateTurningLeft(void)
{
  if ((is_engage_requesting_ == true) ||
      (turn_signal_ != autoware_adapi_v1_msgs::msg::TurnIndicators::LEFT)) {
    return false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_LEFT;
  return true;
}

bool AdSoundManager::changeStateTurningRight(void)
{
  if ((is_engage_requesting_ == true) ||
      (turn_signal_ != autoware_adapi_v1_msgs::msg::TurnIndicators::RIGHT)) {
    return false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_RIGHT;
  return true;
}

bool AdSoundManager::changeStateInformRestart(void)
{
  if (is_engage_requesting_ != true ) {
    return false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART;
  return true;
}

bool AdSoundManager::changeStateRunningTowardStopLine(void)
{
  if ((is_engage_requesting_ == true) ||
      (is_stop_reason_ != true) ||
      (is_surround_obstacle_check_ == true) ||
      (dist_to_stop_pose_ <= dist_to_stop_pose_min_th_)) {
    return false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_STOP_LINE;
  return true;
}

bool AdSoundManager::changeStateRunningTowardObstacle(void)
{
  if ((is_engage_requesting_ == true) ||
      ((is_obstacle_stop_ != true) && (is_detection_area_ != true) && (is_crosswalk_ != true)) ||
      (dist_to_stop_pose_ <= dist_to_stop_pose_min_th_)) {
    return false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_RUNNING_TOWARD_OBSTACLE;
  return true;
}

bool AdSoundManager::changeStateStopDuetoTrafficCondition(void)
{
  if ((is_engage_requesting_ == true) ||
      (is_stop_reason_ != true) ||
      (is_surround_obstacle_check_ == true) || 
      (dist_to_stop_pose_ > dist_to_stop_pose_min_th_)) {
    return false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION;
  return true;
}

bool AdSoundManager::changeStateStopDuetoApproachingObstacle(void)
{
  if ((is_engage_requesting_ == true) ||
      ((is_obstacle_stop_ != true) && (is_detection_area_ != true) && (is_crosswalk_ != true)) ||
      (dist_to_stop_pose_ > dist_to_stop_pose_min_th_)) {
    return false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE;
  return true;
}

bool AdSoundManager::changeStateStopDuetoSurroundingProximity(void)
{
  if ((is_engage_requesting_ == true) ||
      (is_surround_obstacle_check_ != true)) {
    return false;
  }

  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY;
  return true;
}

bool AdSoundManager::changeStateArrivedGoal(void)
{
  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL;
  return true;
}

bool AdSoundManager::updateStateOfInitializingVehicle(void)
{
  if (changeStateDuringWakeUp() == true) {
    return true;
  } else {
    return false;
  }
}

bool AdSoundManager::updateState4WaitingForRoute(void)
{
  if (changeStateDuringReceiveRoute() == true) {
    return true;
  } else {
    return false;
  }
}

bool AdSoundManager::updateState4Planning(void)
{
  if (changeStateDuringReceiveRoute() == true) {
    return true;
  } else {
    return false;
  }
}

bool AdSoundManager::updateState4WaitingForEngage(void)
{
  if ((changeStateWaitingEngageInstruction() == true) ||
      (changeStateWaitingCallPermission() == true) ||
      (changeStateInformEngage() == true) ||
      (changeStateInstructEngage() == true)) {
    return true;
  } else {
    return false;
  }
}

bool AdSoundManager::updateState4Drivig(void)
{
  if ((changeStateRunning() == true) ||
      (changeStateRunningTowardStopLine() == true) ||
      (changeStateRunningTowardObstacle() == true) ||
      (changeStateTurningLeft() == true) ||
      (changeStateTurningRight() == true) ||
      (changeStateStopDuetoTrafficCondition() == true) ||
      (changeStateStopDuetoApproachingObstacle() == true) ||
      (changeStateStopDuetoSurroundingProximity() == true) ||
      (changeStateInformRestart() == true)) {
    return true;
  } else {
    return false;
  }
}

bool AdSoundManager::updateState4ArrivedGoal(void)
{
  if (changeStateArrivedGoal() == true) {
    return true;
  } else {
    return false;
  }
}

void AdSoundManager::changeState()
{
  auto pre_service_layer_state_ = service_layer_state_;

  if (service_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED) {
    changeCheckNodeAlive();

  } else if (em_holding_ == true) {
    // STATE_EMERGENCY_STOP
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;

  } else if (isAutowareStateOfDriving() == true) {
    updateState4Drivig();

  } else if (isAutowareStateOfWaitingForEngage() == true) {
    updateState4WaitingForEngage();

  } else if (isAutowareStateOfPlanning() == true) {
    updateState4Planning();

  } else if (isAutowareStateOfWaitingForRoute() == true) {
    updateState4WaitingForRoute();

  } else if (isAutowareStateOfWaitingForRoute() == true) {
    updateState4WaitingForRoute();

  } else if (isAutowareStateOfInitializingVehicle() == true) {
    updateStateOfInitializingVehicle();

  } else {
    // 上記以外
    service_layer_state_ = 0xFFFF;
  }

  if ((operation_state_.is_stop_mode_available == true) ||
      (operation_state_.is_local_mode_available == true)) {
    control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  } else {
    control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::AUTO;
  }

  if (pre_service_layer_state_ != service_layer_state_) {
    changeSoundState(service_layer_state_, control_layer_state_, false);
  }
}

}  // namespace ad_sound_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ad_sound_manager::AdSoundManager)
