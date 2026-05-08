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
#include <cmath>
#include <limits>
#include <queue>
#include <cstdint>

#include "ad_sound_manager/ad_sound_manager.hpp"

namespace
{

using tier4_external_api_msgs::msg::PlanningFactor;
using tier4_external_api_msgs::msg::PlanningFactorArray;

/** First control point distance (m), or +inf if none. */
double factorPrimaryDistanceM(const PlanningFactor & factor)
{
  if (factor.control_points.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(factor.control_points[0].distance);
}

/**
 * 旧 AutowareStateMachine::getNearestStopReasonWithPriority の優先度に相当。
 * tier4_planning_msgs::StopReason の定数名と planning_factors の behavior_name を対応付け。
 */
int8_t planningBehaviorStopPriority(const std::string & behavior_name)
{
  if (behavior_name == "surround_obstacle_checker" || behavior_name == "surrounding_obstacle") {
    return 1;  // SURROUND_OBSTACLE_CHECK
  }
  if (behavior_name == "obstacle_stop" || behavior_name == "route_obstacle") {
    return 2;  // OBSTACLE_STOP
  }
  if (behavior_name == "user_defined_detection_area") {
    return 3;  // DETECTION_AREA
  }
  if (behavior_name == "virtual_traffic_light") {
    return 4;  // VIRTUAL_TRAFFIC_LIGHT
  }
  if (behavior_name == "stop_sign") {
    return 5;  // STOP_LINE 相当
  }
  return 10;
}

struct FactorStopInfo
{
  std::string behavior_name;
  double distance;
  int8_t priority;
};

/**
 * 全 factors を走査し、旧 getNearestStopReasonWithPriority と同様の比較で最も手前／優先度の高い要因を1つ選ぶ。
 * dist_select_max_m より遠い（>=）要因は除外（旧 dist_to_stop_pose < max と同趣旨）。
 */
std::pair<std::string, double> selectNearestPlanningFactorBehaviorWithPriority(
  const PlanningFactorArray & msg, double dist_select_max_m)
{
  static constexpr double kNearDist = 1e-3;
  auto compare = [](const FactorStopInfo & a, const FactorStopInfo & b) -> bool {
    if (a.distance < kNearDist && b.distance < kNearDist) {
      return a.priority > b.priority;
    }
    return a.distance > b.distance;
  };
  std::priority_queue<FactorStopInfo, std::vector<FactorStopInfo>, decltype(compare)> que(compare);

  for (const auto & factor : msg.factors) {
    if (factor.behavior_name.empty()) {
      continue;
    }
    const double d = factorPrimaryDistanceM(factor);
    if (!std::isfinite(d) || d >= dist_select_max_m) {
      continue;
    }
    que.push(FactorStopInfo{factor.behavior_name, d, planningBehaviorStopPriority(factor.behavior_name)});
  }

  if (que.empty()) {
    return {"", 0.0};
  }
  const FactorStopInfo top = que.top();
  return {top.behavior_name, top.distance};
}

/** API の behavior_name: 仕様名と planning モジュール実名の両方を障害物接近として扱う。 */
bool isObstacleApproachBehaviorName(const std::string & bname)
{
  return bname == "route_obstacle" || bname == "user_defined_detection_area" ||
         bname == "obstacle_stop";
}

}  // namespace

namespace ad_sound_manager
{

bool AdSoundManager::isPlanningSelectedDistAheadOfStopThreshold(double dist_m) const
{
  return std::isfinite(dist_m) && dist_m > stop_approach_dist_threshold_m_;
}

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
  // Note: Use SensorDataQoS (BEST_EFFORT + volatile) to match publisher QoS
  sub_adapi_vehicle_status_ = this->create_subscription<autoware_adapi_v1_msgs::msg::VehicleStatus>(
    "/api/vehicle/status",
    rclcpp::SensorDataQoS(),
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
  // Subscriptions - System
  // ============================================================

  // For: /api/external/get/hazard_status (emergency_holding)
  // Note: Use QoS{1} (volatile) to match publisher QoS
  sub_hazard_status_ = this->create_subscription<tier4_external_api_msgs::msg::HazardStatusStamped>(
    "/api/external/get/hazard_status",
    rclcpp::QoS{1},
    std::bind(&AdSoundManager::callbackHazardStatus, this, std::placeholders::_1)
  );

  // ============================================================
  // Subscriptions - /api/external/get/planning_factors (AW API)
  // ============================================================

  sub_planning_factors_ = this->create_subscription<PlanningFactorArray>(
    "/api/external/get/planning_factors",
    rclcpp::QoS{1},
    std::bind(&AdSoundManager::callbackPlanningFactors, this, std::placeholders::_1)
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
  double stop_dist_to_prohibit_engage = this->declare_parameter<double>(
    "stop_dist_to_prohibit_engage", 0.30);
  // Add a value of 0.05 to `stop_dist_to_prohibit_engage`.
  stop_approach_dist_threshold_m_ = stop_dist_to_prohibit_engage + 0.05;
  planning_factors_selection_dist_max_m_ =
    this->declare_parameter<double>("planning_factors_selection_dist_max_m", 500.0);

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

  // Initialize sound playback flags
  is_playing_wakeup_sound_ = false;
  is_playing_engage_sound_ = false;
  is_playing_restart_sound_ = false;
  is_playing_arrival_sound_ = false;
  has_started_driving_ = false;

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

  // Trigger wakeup sound at startup (same as legacy autoware_state_machine behavior)
  // STATE_UNDEFINED -> STATE_CHECK_NODE_ALIVE
  changeSoundState(
    autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE,
    autoware_state_machine_msgs::msg::StateMachine::MANUAL,
    false);
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
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] callbackVoiceRes: one_play_state=%s",
    serviceLayerStateToString(one_play_state_).c_str());

  // Done information is returned only after one-time playback.
  if (one_play_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL)
  {
    RCLCPP_INFO(this->get_logger(),
      "[DEBUG] Arrival sound completed, transitioning to STATE_DURING_RECEIVE_ROUTE");
    is_playing_arrival_sound_ = false;  // Clear flag
    publishSoundDone();
    // 到着音声再生完了後、STATE_DURING_RECEIVE_ROUTE に遷移（BGM停止）
    changeSoundState(
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE,
      cur_control_layer_state_,
      false);
    return;
  }

  // STATE_CHECK_NODE_ALIVE の音声完了後、明示的に STATE_DURING_WAKEUP に遷移
  // これにより、localization の完了を待つ状態に移行する（旧実装と同等の動作）
  if (one_play_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE)
  {
    RCLCPP_INFO(this->get_logger(),
      "[DEBUG] Wakeup sound completed, transitioning to STATE_DURING_WAKEUP");
    is_playing_wakeup_sound_ = false;  // Clear flag
    publishSoundDone();
    // 起動音声完了後、現在のトピック状態に基づいて次の状態を決定（ステートレス）
    updateAutowareStateFromTopics();
    return;
  }

  // STATE_INFORM_ENGAGE の音声完了後は STATE_INSTRUCT_ENGAGE に遷移
  // （旧実装と同様、走行開始前の待機状態）
  if (one_play_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE)
  {
    RCLCPP_INFO(this->get_logger(),
      "[DEBUG] Engage sound completed, transitioning to STATE_INSTRUCT_ENGAGE");
    is_playing_engage_sound_ = false;  // Clear flag
    post_engage_sound_latched_ = true;
    has_started_driving_ = true;       // 走行セッション: 発進案内音声完了後のみ true
    publishSoundDone();
    // 発進音声完了後、現在の ADAPI 状態に基づいて次の状態を決定（ステートレス）
    updateAutowareStateFromTopics();
    return;
  }

  // STATE_INFORM_RESTART の音声完了後は現在の ADAPI 状態に基づいて遷移
  if (one_play_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART)
  {
    RCLCPP_INFO(this->get_logger(),
      "[DEBUG] Restart sound completed, calling publishSoundDone() and updateAutowareStateFromTopics()");
    is_playing_restart_sound_ = false;  // Clear flag
    has_started_driving_ = true;       // 走行セッション: 再発進案内音声完了後も true
    publishSoundDone();
    // 再発進音声完了後、現在の ADAPI 状態に基づいて次の状態に遷移
    updateAutowareStateFromTopics();
    return;
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

std::string AdSoundManager::serviceLayerStateToString(uint16_t state) const
{
  using StateMachine = autoware_state_machine_msgs::msg::StateMachine;
  switch (state) {
    case StateMachine::STATE_UNDEFINED: return "STATE_UNDEFINED";
    case StateMachine::STATE_DURING_WAKEUP: return "STATE_DURING_WAKEUP";
    case StateMachine::STATE_DURING_CLOSE: return "STATE_DURING_CLOSE";
    case StateMachine::STATE_CHECK_NODE_ALIVE: return "STATE_CHECK_NODE_ALIVE";
    case StateMachine::STATE_DURING_RECEIVE_ROUTE: return "STATE_DURING_RECEIVE_ROUTE";
    case StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION: return "STATE_WAITING_ENGAGE_INSTRUCTION";
    case StateMachine::STATE_WAITING_CALL_PERMISSION: return "STATE_WAITING_CALL_PERMISSION";
    case StateMachine::STATE_RUNNING: return "STATE_RUNNING";
    case StateMachine::STATE_INFORM_ENGAGE: return "STATE_INFORM_ENGAGE";
    case StateMachine::STATE_RUNNING_TOWARD_STOP_LINE: return "STATE_RUNNING_TOWARD_STOP_LINE";
    case StateMachine::STATE_RUNNING_TOWARD_OBSTACLE: return "STATE_RUNNING_TOWARD_OBSTACLE";
    case StateMachine::STATE_INSTRUCT_ENGAGE: return "STATE_INSTRUCT_ENGAGE";
    case StateMachine::STATE_TURNING_LEFT: return "STATE_TURNING_LEFT";
    case StateMachine::STATE_TURNING_RIGHT: return "STATE_TURNING_RIGHT";
    case StateMachine::STATE_DURING_OBSTACLE_AVOIDANCE: return "STATE_DURING_OBSTACLE_AVOIDANCE";
    case StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION: return "STATE_STOP_DUETO_TRAFFIC_CONDITION";
    case StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE: return "STATE_STOP_DUETO_APPROACHING_OBSTACLE";
    case StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY: return "STATE_STOP_DUETO_SURROUNDING_PROXIMITY";
    case StateMachine::STATE_INFORM_RESTART: return "STATE_INFORM_RESTART";
    case StateMachine::STATE_ARRIVED_GOAL: return "STATE_ARRIVED_GOAL";
    case StateMachine::STATE_EMERGENCY_STOP: return "STATE_EMERGENCY_STOP";
    default: return "UNKNOWN(" + std::to_string(state) + ")";
  }
}

std::string AdSoundManager::controlLayerStateToString(uint8_t state) const
{
  using StateMachine = autoware_state_machine_msgs::msg::StateMachine;
  switch (state) {
    case StateMachine::MANUAL: return "MANUAL";
    case StateMachine::AUTO: return "AUTO";
    default: return "UNKNOWN(" + std::to_string(state) + ")";
  }
}

void AdSoundManager::changeSoundState(
  const uint16_t service_layer_state,
  const uint8_t control_layer_state,
  bool force)
{
  // Log every call with argument values (BEFORE updating state)
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] changeSoundState CALLED: arg_service=%s, arg_control=%s, cur_before=%s, force=%d",
    serviceLayerStateToString(service_layer_state).c_str(),
    controlLayerStateToString(control_layer_state).c_str(),
    serviceLayerStateToString(cur_service_layer_state_).c_str(),
    force);

  prev_service_layer_state_ = cur_service_layer_state_;
  prev_control_layer_state_ = cur_control_layer_state_;
  cur_service_layer_state_ = service_layer_state;
  cur_control_layer_state_ = control_layer_state;

  // Log state change only when state actually changed
  if (prev_service_layer_state_ != cur_service_layer_state_) {
    RCLCPP_WARN(this->get_logger(),
      "Service Layer State Changed. %s -> %s",
      serviceLayerStateToString(prev_service_layer_state_).c_str(),
      serviceLayerStateToString(cur_service_layer_state_).c_str());
  }
  if (prev_control_layer_state_ != cur_control_layer_state_) {
    RCLCPP_WARN(this->get_logger(),
      "Control Layer State Changed. %s -> %s",
      controlLayerStateToString(prev_control_layer_state_).c_str(),
      controlLayerStateToString(cur_control_layer_state_).c_str());
  }

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
      is_playing_wakeup_sound_ = true;  // Set flag for stateless state determination
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
      RCLCPP_WARN(this->get_logger(),
        "[DEBUG] STATE_INFORM_ENGAGE case: setting is_playing_engage_sound_=true");
      pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
      continuity_state_ = false;
      one_play_state_ = cur_service_layer_state_;
      is_playing_engage_sound_ = true;  // Set flag for stateless state determination
      playOneshotVoice(sound_filename_start_, (pre_sound_type != PreSoundType::SOUND_NONE));
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART:
      RCLCPP_WARN(this->get_logger(),
        "[DEBUG] STATE_INFORM_RESTART case: setting is_playing_restart_sound_=true");
      pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
      continuity_state_ = false;
      one_play_state_ = cur_service_layer_state_;
      is_playing_restart_sound_ = true;  // Set flag for stateless state determination
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
      is_playing_arrival_sound_ = true;  // Set flag for stateless state determination
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

std::string AdSoundManager::motionStateToString(uint16_t state) const
{
  using MotionState = autoware_adapi_v1_msgs::msg::MotionState;
  switch (state) {
    case MotionState::UNKNOWN: return "UNKNOWN";
    case MotionState::STOPPED: return "STOPPED";
    case MotionState::STARTING: return "STARTING";
    case MotionState::MOVING: return "MOVING";
    default: return "INVALID(" + std::to_string(state) + ")";
  }
}

void AdSoundManager::callbackMotionState(
  const autoware_adapi_v1_msgs::msg::MotionState::ConstSharedPtr msg)
{
  using MotionState = autoware_adapi_v1_msgs::msg::MotionState;
  using LocalizationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;

  prev_motion_state_ = motion_state_;
  motion_state_ = *msg;

  // Detect restart: STOPPED -> MOVING transition after driving has started
  // This handles the case where STARTING state is skipped
  if (prev_motion_state_.state == MotionState::STOPPED &&
      motion_state_.state == MotionState::MOVING &&
      has_started_driving_ &&
      !is_playing_restart_sound_)
  {
    RCLCPP_WARN(this->get_logger(),
      "[DEBUG] Detected restart (STOPPED->MOVING), triggering restart sound");
    is_playing_restart_sound_ = true;
  }

  // has_started_driving_ / driving_session_had_moving_ は motion ではいじらない。

  // 制御は最初から ON で control_rising が無い PSim 等: ルート未 SET の短い MOVING のあと停止したら pending
  // （route SET 中の MOVING→STOPPED は別経路・テスト間の transient_local 残りと区別するためここでは立てない）
  {
    using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
    using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
    if (prev_motion_state_.state == MotionState::MOVING &&
        motion_state_.state == MotionState::STOPPED &&
        route_state_.state != RouteState::SET &&
        operation_mode_state_.is_autoware_control_enabled &&
        operation_mode_state_.mode == OperationModeState::AUTONOMOUS &&
        localization_state_.state == LocalizationState::INITIALIZED &&
        !emergency_holding_ &&
        !has_started_driving_ &&
        !is_playing_engage_sound_)
    {
      pending_autonomous_control_inform_engage_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "[DEBUG] MOVING->STOPPED while route not SET (AUTONOMOUS + Autoware control): "
        "pending STATE_INFORM_ENGAGE after route SET (e.g. obstacle creep then mission)");
    }
  }

  if (prev_motion_state_.state != motion_state_.state) {
    RCLCPP_WARN(this->get_logger(),
      "[DEBUG] MotionState Changed: %s -> %s (has_started_driving=%d)",
      motionStateToString(prev_motion_state_.state).c_str(),
      motionStateToString(motion_state_.state).c_str(),
      has_started_driving_);
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

std::string AdSoundManager::routeStateToString(uint16_t state) const
{
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  switch (state) {
    case RouteState::UNKNOWN: return "UNKNOWN";
    case RouteState::UNSET: return "UNSET";
    case RouteState::SET: return "SET";
    case RouteState::ARRIVED: return "ARRIVED";
    case RouteState::CHANGING: return "CHANGING";
    default: return "INVALID(" + std::to_string(state) + ")";
  }
}

std::string AdSoundManager::localizationStateToString(uint16_t state) const
{
  using LocalizationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  switch (state) {
    case LocalizationState::UNKNOWN: return "UNKNOWN";
    case LocalizationState::UNINITIALIZED: return "UNINITIALIZED";
    case LocalizationState::INITIALIZING: return "INITIALIZING";
    case LocalizationState::INITIALIZED: return "INITIALIZED";
    default: return "INVALID(" + std::to_string(state) + ")";
  }
}

void AdSoundManager::callbackRouteState(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;

  prev_route_state_ = route_state_;
  route_state_ = *msg;

  // 新規トリップ: 発進系フラグをリセット（has_started は update 先頭の同期でも落とす）
  if (route_state_.state == RouteState::UNSET || route_state_.state == RouteState::UNKNOWN) {
    post_engage_sound_latched_ = false;
    pending_autonomous_control_inform_engage_ = false;
    planning_selected_stop_reason_initialized_ = false;
    cached_planning_selected_nearest_ = {"", 0.0};
    driving_session_had_moving_ = false;
  }

  if (prev_route_state_.state != route_state_.state) {
    RCLCPP_WARN(this->get_logger(),
      "[DEBUG] RouteState Changed: %s -> %s (has_started_driving=%d)",
      routeStateToString(prev_route_state_.state).c_str(),
      routeStateToString(route_state_.state).c_str(),
      has_started_driving_);
    updateAutowareStateFromTopics();
  }
}

void AdSoundManager::callbackLocalizationState(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg)
{
  prev_localization_state_ = localization_state_;
  localization_state_ = *msg;

  if (prev_localization_state_.state != localization_state_.state) {
    RCLCPP_WARN(this->get_logger(),
      "[DEBUG] LocalizationState Changed: %s -> %s",
      localizationStateToString(prev_localization_state_.state).c_str(),
      localizationStateToString(localization_state_.state).c_str());
    updateAutowareStateFromTopics();
  }
}

void AdSoundManager::callbackOperationModeState(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  using autoware_adapi_v1_msgs::msg::OperationModeState;
  using autoware_adapi_v1_msgs::msg::RouteState;
  using autoware_adapi_v1_msgs::msg::MotionState;

  prev_operation_mode_state_ = operation_mode_state_;
  operation_mode_state_ = *msg;

  // Call updateAutowareStateFromTopics() when mode or control_enabled changes
  if (prev_operation_mode_state_.mode != operation_mode_state_.mode ||
      prev_operation_mode_state_.is_autoware_control_enabled != operation_mode_state_.is_autoware_control_enabled)
  {
    // 障害物前などで STARTING が付かず停止のまま制御が入ると INSTRUCT に直行し INFORM 音声が出ない。
    // 停止中の立ち上がりは pending で INFORM を挟む。
    const bool control_rising =
      !prev_operation_mode_state_.is_autoware_control_enabled &&
      operation_mode_state_.is_autoware_control_enabled;
    const bool mode_became_autonomous =
      prev_operation_mode_state_.mode != OperationModeState::AUTONOMOUS &&
      operation_mode_state_.mode == OperationModeState::AUTONOMOUS;
    if ((control_rising || mode_became_autonomous) &&
        operation_mode_state_.is_autoware_control_enabled &&
        operation_mode_state_.mode == OperationModeState::AUTONOMOUS &&
        route_state_.state == RouteState::SET &&
        motion_state_.state == MotionState::STOPPED &&
        !has_started_driving_)
    {
      pending_autonomous_control_inform_engage_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "[DEBUG] Autoware AUTONOMOUS+control while STOPPED (route SET): "
        "pending STATE_INFORM_ENGAGE (control edge or mode->AUTONOMOUS; e.g. PSim, obstacle ahead)");
    }

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
// Callback functions - System
// ============================================================

void AdSoundManager::callbackHazardStatus(
  const tier4_external_api_msgs::msg::HazardStatusStamped::ConstSharedPtr msg)
{
  bool prev_emergency_holding = emergency_holding_;
  emergency_holding_ = msg->status.emergency_holding;

  // Call updateAutowareStateFromTopics() when emergency_holding changes
  if (prev_emergency_holding != emergency_holding_) {
    RCLCPP_WARN(this->get_logger(),
      "[DEBUG] emergency_holding Changed: %d -> %d",
      prev_emergency_holding, emergency_holding_);
    updateAutowareStateFromTopics();
  }
}

// ============================================================
// Callback functions - /api/external/get/planning_factors
// ============================================================

void AdSoundManager::callbackPlanningFactors(
  const tier4_external_api_msgs::msg::PlanningFactorArray::ConstSharedPtr msg)
{
  const std::pair<std::string, double> nearest = selectNearestPlanningFactorBehaviorWithPriority(
    *msg, planning_factors_selection_dist_max_m_);
  const bool dist_ahead_curr = isPlanningSelectedDistAheadOfStopThreshold(nearest.second);
  const bool dist_ahead_cached =
    isPlanningSelectedDistAheadOfStopThreshold(cached_planning_selected_nearest_.second);

  // behavior も閾値前後も変わらなければ update 不要（cached の距離は更新しない）
  if (planning_selected_stop_reason_initialized_ &&
      nearest.first == cached_planning_selected_nearest_.first &&
      dist_ahead_curr == dist_ahead_cached)
  {
    return;
  }

  cached_planning_selected_nearest_ = nearest;
  planning_selected_stop_reason_initialized_ = true;
  updateAutowareStateFromTopics();
}

// ============================================================
// State conversion function
// ============================================================

void AdSoundManager::updateAutowareStateFromTopics(void)
{
  // This function converts new ADAPI v1 states to legacy service_layer_state and control_layer_state
  // using a STATELESS approach based on current flags and topic states.
  // Priority order follows the design document (Section 5.5).

  using StateMachine = autoware_state_machine_msgs::msg::StateMachine;
  using MotionState = autoware_adapi_v1_msgs::msg::MotionState;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using LocalizationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using TurnIndicators = autoware_adapi_v1_msgs::msg::TurnIndicators;

  const bool had_started_driving_before = has_started_driving_;

  // 走行セッションフラグ: 案内音声で true にしたあとも、ルート・定位・非常停止で false
  if (emergency_holding_ ||
      route_state_.state != RouteState::SET ||
      localization_state_.state != LocalizationState::INITIALIZED)
  {
    has_started_driving_ = false;
    driving_session_had_moving_ = false;
  }
  // pending は route 未 SET の間も保持する（UNSET 中に MOVING→STOPPED してから SET する PSim 向け）。
  // クリアは route UNSET/UNKNOWN の callback と、定位未済・非常停止のみ。
  if (emergency_holding_ ||
      localization_state_.state != LocalizationState::INITIALIZED)
  {
    pending_autonomous_control_inform_engage_ = false;
  }

  // Debug: log input states
  RCLCPP_INFO(this->get_logger(),
    "[DEBUG] updateAutowareStateFromTopics: localization=%u, route=%u, motion=%u, control_enabled=%d, "
    "wakeup=%d, engage=%d, restart=%d, arrival=%d, has_started=%d session_had_moving=%d",
    localization_state_.state, route_state_.state, motion_state_.state,
    operation_mode_state_.is_autoware_control_enabled,
    is_playing_wakeup_sound_, is_playing_engage_sound_, is_playing_restart_sound_,
    is_playing_arrival_sound_, has_started_driving_, driving_session_had_moving_ ? 1 : 0);

  // ============================================================
  // Derive control_layer_state
  // ============================================================
  uint8_t control_layer_state = StateMachine::MANUAL;
  if (operation_mode_state_.is_autoware_control_enabled) {
    control_layer_state = StateMachine::AUTO;
  }

  // ============================================================
  // Derive service_layer_state (STATELESS priority order)
  // Based on design document Section 5.5
  // ============================================================
  uint16_t service_layer_state = StateMachine::STATE_UNDEFINED;

  // 全 factors を旧 getNearestStopReasonWithPriority 相当で集約（callback でキャッシュ済み）
  const std::string & planning_sel_name = cached_planning_selected_nearest_.first;
  const double planning_sel_dist = cached_planning_selected_nearest_.second;
  const bool planning_sel_dist_ahead = isPlanningSelectedDistAheadOfStopThreshold(planning_sel_dist);
  const bool planning_p14_stop_factor =
    isObstacleApproachBehaviorName(planning_sel_name) ||
    ((planning_sel_name == "surround_obstacle_checker" || planning_sel_name == "surrounding_obstacle") &&
     post_engage_sound_latched_);

  // Priority 1: Wakeup sound playing (highest priority)
  if (is_playing_wakeup_sound_) {
    service_layer_state = StateMachine::STATE_CHECK_NODE_ALIVE;
  }
  // Priority 2: Localization not initialized (STATE_DURING_WAKEUP)
  // Note: Emergency stop is NOT allowed during wakeup phase (legacy behavior)
  else if (localization_state_.state != LocalizationState::INITIALIZED) {
    service_layer_state = StateMachine::STATE_DURING_WAKEUP;
  }
  // Priority 3: Emergency stop (only after wakeup phase is complete)
  else if (emergency_holding_) {
    service_layer_state = StateMachine::STATE_EMERGENCY_STOP;
  }
  // Priority 4: Arrival sound playing
  else if (is_playing_arrival_sound_) {
    service_layer_state = StateMachine::STATE_ARRIVED_GOAL;
  }
  // Priority 5: Engage sound playing
  else if (is_playing_engage_sound_) {
    service_layer_state = StateMachine::STATE_INFORM_ENGAGE;
  }
  // Priority 6: Restart sound playing
  else if (is_playing_restart_sound_) {
    service_layer_state = StateMachine::STATE_INFORM_RESTART;
  }
  // Priority 7: Arrived at goal
  else if (route_state_.state == RouteState::ARRIVED) {
    service_layer_state = StateMachine::STATE_ARRIVED_GOAL;
  }
  // Priority 8: Route not set, unknown, or changing
  // MOVING の例外は付けない。route UNKNOWN + localization 済 + MOVING で P13 に落ちると
  // ルート設定・発進前に STATE_RUNNING（走行系 BGM）になってしまう（psim.log 参照）。
  else if (route_state_.state == RouteState::UNSET ||
           route_state_.state == RouteState::UNKNOWN ||
           route_state_.state == RouteState::CHANGING)
  {
    service_layer_state = StateMachine::STATE_DURING_RECEIVE_ROUTE;
  }
  // Priority 9: Waiting for call permission (go_interface)
  else if (route_state_.state == RouteState::SET &&
           motion_state_.state == MotionState::STOPPED &&
           !has_started_driving_ &&
           go_interface_vehicle_status_.voice_flg &&
           go_interface_vehicle_status_.lock_flg)
  {
    service_layer_state = StateMachine::STATE_WAITING_CALL_PERMISSION;
  }
  // Priority 9b: 制御立ち上がりで STOPPED のまま（障害物前で STARTING が出ない）→ 発進案内音声を挟む
  // P10 より前に置く（has_started がまだ false のため P10 と両立する）
  // engage 相当（AUTONOMOUS かつ Autoware 制御 ON）のときだけ INFORM を選び、pending は満たすまで保持する
  else if (pending_autonomous_control_inform_engage_ &&
           route_state_.state == RouteState::SET &&
           motion_state_.state == MotionState::STOPPED &&
           operation_mode_state_.is_autoware_control_enabled &&
           operation_mode_state_.mode == OperationModeState::AUTONOMOUS &&
           !has_started_driving_ &&
           !is_playing_engage_sound_)
  {
    service_layer_state = StateMachine::STATE_INFORM_ENGAGE;
    pending_autonomous_control_inform_engage_ = false;
  }
  // Priority 10: Waiting for engage instruction（発進音声が未完了のときのみ。完了後は P11 / P14 へ）
  else if (route_state_.state == RouteState::SET &&
           motion_state_.state == MotionState::STOPPED &&
           !has_started_driving_)
  {
    service_layer_state = StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;
  }
  // Priority 10b: 走行セッション中の停止（一度 MOVING したあと、または planning が障害物／周辺近接）
  // P11 より前（発進直後に障害で止まると has_started のみ true のため）
  else if (motion_state_.state == MotionState::STOPPED &&
           has_started_driving_ &&
           route_state_.state == RouteState::SET &&
           (driving_session_had_moving_ || planning_p14_stop_factor))
  {
    const std::string & bname = planning_sel_name;
    if ((bname == "surround_obstacle_checker" || bname == "surrounding_obstacle") &&
      post_engage_sound_latched_)
    {
      service_layer_state = StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY;
    } else if (isObstacleApproachBehaviorName(bname)) {
      service_layer_state = StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE;
    } else {
      // "virtual_traffic_light", その他、要因なし
      service_layer_state = StateMachine::STATE_STOP_DUETO_TRAFFIC_CONDITION;
    }
  }
  // Priority 11: INSTRUCT_ENGAGE（発進音声済み・初回 MOVING 前・障害 planning なしで停止中）
  else if (has_started_driving_ &&
           route_state_.state == RouteState::SET &&
           motion_state_.state == MotionState::STOPPED &&
           !driving_session_had_moving_) {
    service_layer_state = StateMachine::STATE_INSTRUCT_ENGAGE;
  }
  // Priority 12: Starting - initial engage (motion=STARTING, not yet driving session)
  else if (motion_state_.state == MotionState::STARTING && !has_started_driving_) {
    service_layer_state = StateMachine::STATE_INFORM_ENGAGE;
  }
  // Priority 12b: Starting - restart after stop (motion=STARTING, already driving)
  else if (motion_state_.state == MotionState::STARTING && has_started_driving_) {
    service_layer_state = StateMachine::STATE_INFORM_RESTART;
  }
  // Priority 12c: ルート SET 時に既に MOVING（UNSET 中に MOVING が先に立つ PSim 等）で発進前のときは
  // RUNNING にせず発進案内へ（has_started は route 非 SET の MOVING では立てない）
  else if (motion_state_.state == MotionState::MOVING &&
           route_state_.state == RouteState::SET &&
           localization_state_.state == LocalizationState::INITIALIZED &&
           operation_mode_state_.is_autoware_control_enabled &&
           !has_started_driving_ &&
           !is_playing_engage_sound_)
  {
    service_layer_state = StateMachine::STATE_INFORM_ENGAGE;
  }
  // Priority 13: Moving under Autoware control（route は SET のみ。未 SET は P8）
  // planning に応じて接近・周辺を表現し、該当なければ RUNNING / ウインカー
  else if (motion_state_.state == MotionState::MOVING &&
           localization_state_.state == LocalizationState::INITIALIZED &&
           operation_mode_state_.is_autoware_control_enabled &&
           route_state_.state == RouteState::SET)
  {
    driving_session_had_moving_ = true;

    const std::string & bname = planning_sel_name;

    if ((bname == "surround_obstacle_checker" || bname == "surrounding_obstacle") &&
      post_engage_sound_latched_)
    {
      service_layer_state = StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY;
    } else if (planning_sel_dist_ahead && isObstacleApproachBehaviorName(bname)) {
      service_layer_state = StateMachine::STATE_RUNNING_TOWARD_OBSTACLE;
    } else if (planning_sel_dist_ahead && bname == "virtual_traffic_light") {
      service_layer_state = StateMachine::STATE_RUNNING_TOWARD_STOP_LINE;
    } else if (adapi_vehicle_status_.turn_indicators.status == TurnIndicators::LEFT) {
      service_layer_state = StateMachine::STATE_TURNING_LEFT;
    } else if (adapi_vehicle_status_.turn_indicators.status == TurnIndicators::RIGHT) {
      service_layer_state = StateMachine::STATE_TURNING_RIGHT;
    } else {
      service_layer_state = StateMachine::STATE_RUNNING;
    }
  }
  // Priority 15: Default
  else {
    service_layer_state = StateMachine::STATE_UNDEFINED;
  }

  // surround_obstacle_checker 診断（planning_factors は高頻度のため throttle）
  {
    const std::string & sb = planning_sel_name;
    if (sb == "surround_obstacle_checker" || sb == "surrounding_obstacle") {
      const double d0 = planning_sel_dist;
      const int is_surround_service =
        (service_layer_state == StateMachine::STATE_STOP_DUETO_SURROUNDING_PROXIMITY) ? 1 : 0;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[surround_obstacle_checker_diag] motion=%u route=%u session_had_moving=%d "
        "post_engage_sound_latched=%d had_started_before=%d has_started_after=%d dist0=%.3f "
        "stop_approach_th=%.2f derived_service=%s (SURROUNDING_PROXIMITY_service=%d; "
        "周辺近接へは post_engage_sound_latched=1 のときのみ)",
        motion_state_.state, route_state_.state, driving_session_had_moving_ ? 1 : 0,
        post_engage_sound_latched_ ? 1 : 0,
        had_started_driving_before ? 1 : 0, has_started_driving_ ? 1 : 0,
        d0, stop_approach_dist_threshold_m_,
        serviceLayerStateToString(service_layer_state).c_str(), is_surround_service);
    }
  }

  // 詳細デバッグログ：全トピック情報を時系列で出力
  RCLCPP_WARN(this->get_logger(),
    "\n========== [STATE TRANSITION LOG] ==========\n"
    "  motion_state: %s (prev: %s)\n"
    "  route_state: %s\n"
    "  localization_state: %s\n"
    "  turn_indicators: %d\n"
    "  has_started_driving: %s\n"
    "  is_playing_restart_sound: %s\n"
    "  is_playing_engage_sound: %s\n"
    "  driving_session_had_moving: %s\n"
    "  voice_flg: %s, lock_flg: %s\n"
    "  --> derived_service: %s\n"
    "  --> derived_control: %s\n"
    "=============================================",
    motionStateToString(motion_state_.state).c_str(),
    motionStateToString(prev_motion_state_.state).c_str(),
    routeStateToString(route_state_.state).c_str(),
    localizationStateToString(localization_state_.state).c_str(),
    adapi_vehicle_status_.turn_indicators.status,
    has_started_driving_ ? "true" : "false",
    is_playing_restart_sound_ ? "true" : "false",
    is_playing_engage_sound_ ? "true" : "false",
    driving_session_had_moving_ ? "true" : "false",
    go_interface_vehicle_status_.voice_flg ? "true" : "false",
    go_interface_vehicle_status_.lock_flg ? "true" : "false",
    serviceLayerStateToString(service_layer_state).c_str(),
    controlLayerStateToString(control_layer_state).c_str());

  changeSoundState(service_layer_state, control_layer_state, false);
}

}  // namespace ad_sound_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ad_sound_manager::AdSoundManager)
