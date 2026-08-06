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
#include <cstdlib>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ad_sound_manager/ad_sound_manager.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr std::chrono::seconds kDirectionSoundDelay{1};
constexpr std::chrono::seconds kDistanceSoundDelay{2};
constexpr std::chrono::seconds kPointSoundCooldown{3};
// Distance buckets are inclusive at the upper bound.
constexpr float kDistanceThreshold3M = 3.0F;
constexpr float kDistanceThreshold5M = 5.0F;
constexpr float kDistanceThreshold10M = 10.0F;
constexpr float kDistanceThreshold15M = 15.0F;

float quatToYaw(const float qx, const float qy, const float qz, const float qw)
{
  const float sin_yaw_cosp = 2.0F * (qw * qz + qx * qy);
  const float cos_yaw_cosp = 1.0F - 2.0F * (qy * qy + qz * qz);
  return std::atan2(sin_yaw_cosp, cos_yaw_cosp);
}

void worldToVehicleRelative(
  const float car_x, const float car_y, const float car_z,
  const float yaw,
  const float obj_x, const float obj_y, const float obj_z,
  float * rel_x, float * rel_y, float * rel_z)
{
  const float dx = obj_x - car_x;
  const float dy = obj_y - car_y;
  const float dz = obj_z - car_z;

  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);

  *rel_x = cos_yaw * dx + sin_yaw * dy;
  *rel_y = -sin_yaw * dx + cos_yaw * dy;
  *rel_z = dz;
}

std::string classifyDirection8(const float x, const float y)
{
  const float angle = static_cast<float>(std::atan2(y, x) * kRadToDeg);

  if (angle >= -22.5F && angle <= 22.5F) {
    return "front";
  } else if (angle > 22.5F && angle <= 67.5F) {
    return "front_left";
  } else if (angle > 67.5F && angle <= 112.5F) {
    return "left";
  } else if (angle > 112.5F && angle <= 157.5F) {
    return "rear_left";
  } else if (angle > 157.5F || angle <= -157.5F) {
    return "rear";
  } else if (angle > -157.5F && angle <= -112.5F) {
    return "rear_right";
  } else if (angle > -112.5F && angle <= -67.5F) {
    return "right";
  }

  return "front_right";
}

}  // namespace

namespace ad_sound_manager
{

AdSoundManager::AdSoundManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("ad_sound_manager", options)
{
  sub_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateMachine>(
    "/autoware_state_machine/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::callbackAutowareStateMachine, this, std::placeholders::_1)
  );

  sub_awapi_vehicle_state_ = this->create_subscription<tier4_api_msgs::msg::AwapiVehicleStatus>(
    "/awapi/vehicle/get/status",
    rclcpp::QoS{1},
    std::bind(&AdSoundManager::callbackAwapiVehicleState, this, std::placeholders::_1)
  );

  sub_stop_reasons_ = this->create_subscription<tier4_planning_msgs::msg::StopReasonArray>(
    "/planning/scenario_planning/status/stop_reasons",
    rclcpp::QoS{3},
    std::bind(&AdSoundManager::callbackStopReasons, this, std::placeholders::_1)
  );

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
  sound_filename_turn_left_ = this->declare_parameter<std::string>("sound_filename_turn_left", "");
  sound_filename_turn_right_ = this->declare_parameter<std::string>("sound_filename_turn_right", "");
  sound_filename_bgm_ = this->declare_parameter<std::string>("sound_filename_bgm", "");
  sound_filename_obstacle_ = this->declare_parameter<std::string>("sound_filename_obstacle", "");
  sound_filename_wakeup_ = this->declare_parameter<std::string>("sound_filename_wakeup", "");
  sound_filename_leave_ = this->declare_parameter<std::string>("sound_filename_leave", "");
  sound_filename_arrival_ = this->declare_parameter<std::string>("sound_filename_arrival", "");
  sound_filename_call_ = this->declare_parameter<std::string>("sound_filename_call", "");
  sound_filename_alert_imu_initialize_ = this->declare_parameter<std::string>("sound_filename_alert_imu_initialize", "");
  sound_filename_3m_ = this->declare_parameter<std::string>("sound_filename_3m", "");
  sound_filename_5m_ = this->declare_parameter<std::string>("sound_filename_5m", "");
  sound_filename_10m_ = this->declare_parameter<std::string>("sound_filename_10m", "");
  sound_filename_15m_ = this->declare_parameter<std::string>("sound_filename_15m", "");
  sound_filename_over_15m_ = this->declare_parameter<std::string>("sound_filename_15m_over", "");
  sound_filename_front_ = this->declare_parameter<std::string>("sound_filename_front", "");
  sound_filename_front_left_ = this->declare_parameter<std::string>("sound_filename_front_left", "");
  sound_filename_front_right_ = this->declare_parameter<std::string>("sound_filename_front_right", "");
  sound_filename_rear_ = this->declare_parameter<std::string>("sound_filename_rear", "");
  sound_filename_rear_left_ = this->declare_parameter<std::string>("sound_filename_rear_left", "");
  sound_filename_rear_right_ = this->declare_parameter<std::string>("sound_filename_rear_right", "");
  sound_filename_left_ = this->declare_parameter<std::string>("sound_filename_left", "");
  sound_filename_right_ = this->declare_parameter<std::string>("sound_filename_right", "");
  sound_filename_detecting_ = this->declare_parameter<std::string>("sound_filename_detecting", "");
  sound_filename_detecting_route_ = this->declare_parameter<std::string>("sound_filename_detecting_route", "");
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
  last_stop_reasons_ = nullptr;
  processed_stop_reasons_ = nullptr;

  std::string sound_directory_path =
    sound_directory_path_.insert(sound_directory_path_.size(), "/");

  sound_filename_avoid_ = sound_directory_path + sound_filename_avoid_;
  sound_filename_start_ = sound_directory_path + sound_filename_start_;
  sound_filename_turn_left_ = sound_directory_path + sound_filename_turn_left_;
  sound_filename_turn_right_ = sound_directory_path + sound_filename_turn_right_;
  sound_filename_bgm_ = sound_directory_path + sound_filename_bgm_;
  sound_filename_obstacle_ = sound_directory_path + sound_filename_obstacle_;
  sound_filename_wakeup_ = sound_directory_path + sound_filename_wakeup_;
  sound_filename_leave_ = sound_directory_path + sound_filename_leave_;
  sound_filename_arrival_ = sound_directory_path + sound_filename_arrival_;
  sound_filename_call_ = sound_directory_path + sound_filename_call_;
  sound_filename_alert_imu_initialize_ = sound_directory_path + sound_filename_alert_imu_initialize_;
  sound_filename_3m_ = sound_directory_path + sound_filename_3m_;
  sound_filename_5m_ = sound_directory_path + sound_filename_5m_;
  sound_filename_10m_ = sound_directory_path + sound_filename_10m_;
  sound_filename_15m_ = sound_directory_path + sound_filename_15m_;
  sound_filename_over_15m_ = sound_directory_path + sound_filename_over_15m_;
  sound_filename_front_ = sound_directory_path + sound_filename_front_;
  sound_filename_front_left_ = sound_directory_path + sound_filename_front_left_;
  sound_filename_front_right_ = sound_directory_path + sound_filename_front_right_;
  sound_filename_rear_ = sound_directory_path + sound_filename_rear_;
  sound_filename_rear_left_ = sound_directory_path + sound_filename_rear_left_;
  sound_filename_rear_right_ = sound_directory_path + sound_filename_rear_right_;
  sound_filename_left_ = sound_directory_path + sound_filename_left_;
  sound_filename_right_ = sound_directory_path + sound_filename_right_;
  sound_filename_detecting_ = sound_directory_path + sound_filename_detecting_;
  sound_filename_detecting_route_ = sound_directory_path + sound_filename_detecting_route_;

  // Check for the existence of audio files.
  makeFullPathWithFileCheck(sound_filename_avoid_);
  makeFullPathWithFileCheck(sound_filename_start_);
  makeFullPathWithFileCheck(sound_filename_turn_left_);
  makeFullPathWithFileCheck(sound_filename_turn_right_);
  makeFullPathWithFileCheck(sound_filename_bgm_);
  makeFullPathWithFileCheck(sound_filename_obstacle_);
  makeFullPathWithFileCheck(sound_filename_wakeup_);
  makeFullPathWithFileCheck(sound_filename_leave_);
  makeFullPathWithFileCheck(sound_filename_arrival_);
  makeFullPathWithFileCheck(sound_filename_call_);
  makeFullPathWithFileCheck(sound_filename_alert_imu_initialize_);
  makeFullPathWithFileCheck(sound_filename_3m_);
  makeFullPathWithFileCheck(sound_filename_5m_);
  makeFullPathWithFileCheck(sound_filename_10m_);
  makeFullPathWithFileCheck(sound_filename_15m_);
  makeFullPathWithFileCheck(sound_filename_over_15m_);
  makeFullPathWithFileCheck(sound_filename_front_);
  makeFullPathWithFileCheck(sound_filename_front_left_);
  makeFullPathWithFileCheck(sound_filename_front_right_);
  makeFullPathWithFileCheck(sound_filename_rear_);
  makeFullPathWithFileCheck(sound_filename_rear_left_);
  makeFullPathWithFileCheck(sound_filename_rear_right_);
  makeFullPathWithFileCheck(sound_filename_left_);
  makeFullPathWithFileCheck(sound_filename_right_);
  makeFullPathWithFileCheck(sound_filename_detecting_);
  makeFullPathWithFileCheck(sound_filename_detecting_route_);
}

AdSoundManager::~AdSoundManager()
{
  stopStopReasonPlayback();
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

void AdSoundManager::callbackAutowareStateMachine(
  const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[AdSoundManager::callback]service_layer_state: %u, control_layer_state: %u",
    msg->service_layer_state,
    msg->control_layer_state);

  changeSoundState(msg->service_layer_state, msg->control_layer_state, false);
    if (msg->service_layer_state !=
      autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE)
    {
      stopStopReasonPlayback();
    }
}

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

void AdSoundManager::callbackAwapiVehicleState(
  const tier4_api_msgs::msg::AwapiVehicleStatus::ConstSharedPtr msg)
{
  if (turn_signal_ != msg->turn_signal) {
    turn_signal_ = msg->turn_signal;
    changeSoundState(cur_service_layer_state_, cur_control_layer_state_, true);
  }
}

void AdSoundManager::callbackStopReasons(
  const tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr msg)
{
  if (msg->stop_reasons.empty()) {
    return;
  }

  last_stop_reasons_ = msg;
  if (cur_service_layer_state_ ==
    autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE)
  {
    requestStopReasonRelativePositionSounds(last_stop_reasons_);
  }
}

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

void AdSoundManager::playStopReasonRelativePositionSounds(
  const tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg)
{
  requestStopReasonRelativePositionSounds(msg);
}

void AdSoundManager::requestStopReasonRelativePositionSounds(
  const tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg)
{
  if (msg == nullptr || msg->stop_reasons.empty()) {
    return;
  }

  if (processed_stop_reasons_ == msg) {
    return;
  }

  stopStopReasonPlayback();
  processed_stop_reasons_ = msg;

  {
    std::lock_guard<std::mutex> lock(stop_reason_playback_mutex_);
    stop_reason_playback_cancel_requested_ = false;
  }

  stop_reason_playback_thread_ = std::thread(
    &AdSoundManager::playStopReasonRelativePositionSoundsWorker, this, msg);
}

void AdSoundManager::playStopReasonRelativePositionSoundsWorker(
  const tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg)
{
  if (msg == nullptr) {
    return;
  }

  const auto pre_sound_type = checkPreSoundType();
  const bool is_cut_in_voice =
    (pre_sound_type == PreSoundType::TURN_LEFTRIGHT_SOUND) ||
    (pre_sound_type == PreSoundType::STOP_REASON_SOUND);

  for (const auto & stop_reason : msg->stop_reasons) {
    const auto & reason_name = stop_reason.reason;

    for (const auto & factor : stop_reason.stop_factors) {
      const auto & stop_pose = factor.stop_pose;
      const auto & vehicle_pos = stop_pose.position;
      const auto & vehicle_ori = stop_pose.orientation;

      const float yaw = quatToYaw(
        static_cast<float>(vehicle_ori.x),
        static_cast<float>(vehicle_ori.y),
        static_cast<float>(vehicle_ori.z),
        static_cast<float>(vehicle_ori.w));

      for (const auto & p : factor.stop_factor_points) {
        float rel_x = 0.0F;
        float rel_y = 0.0F;
        float rel_z = 0.0F;
        worldToVehicleRelative(
          static_cast<float>(vehicle_pos.x),
          static_cast<float>(vehicle_pos.y),
          static_cast<float>(vehicle_pos.z),
          yaw,
          static_cast<float>(p.x),
          static_cast<float>(p.y),
          static_cast<float>(p.z),
          &rel_x, &rel_y, &rel_z);

        const std::string direction = classifyDirection8(rel_x, rel_y);
        const float distance = std::hypot(rel_x, rel_y);
        const float angle = std::atan2(rel_y, rel_x) * static_cast<float>(kRadToDeg);

        RCLCPP_INFO(
          this->get_logger(),
          "[stop reasons] reason=%s, %s, distance=%.3f m, angle=%.1f deg",
          reason_name.c_str(), direction.c_str(), distance, angle);

        if (reason_name == "DetectionArea") {
          if (direction == "front") {
            playOneshotVoice(sound_filename_front_, is_cut_in_voice);
          } else if (direction == "front_left") {
            playOneshotVoice(sound_filename_front_left_, is_cut_in_voice);
          } else if (direction == "left") {
            playOneshotVoice(sound_filename_left_, is_cut_in_voice);
          } else if (direction == "rear_left") {
            playOneshotVoice(sound_filename_rear_left_, is_cut_in_voice);
          } else if (direction == "rear") {
            playOneshotVoice(sound_filename_rear_, is_cut_in_voice);
          } else if (direction == "rear_right") {
            playOneshotVoice(sound_filename_rear_right_, is_cut_in_voice);
          } else if (direction == "right") {
            playOneshotVoice(sound_filename_right_, is_cut_in_voice);
          } else {
            playOneshotVoice(sound_filename_front_right_, is_cut_in_voice);
          }
          if (!waitForStopReasonPlaybackDelay(kDirectionSoundDelay)) {
            return;
          }
          // Preserve the existing boundary behavior: 3.0 m and above map to the 5 m bucket.
          if (distance < kDistanceThreshold3M) {
            playOneshotVoice(sound_filename_3m_);
          } else if (distance <= kDistanceThreshold5M) {
            playOneshotVoice(sound_filename_5m_);
          } else if (distance <= kDistanceThreshold10M) {
            playOneshotVoice(sound_filename_10m_);
          } else if (distance <= kDistanceThreshold15M) {
            playOneshotVoice(sound_filename_15m_);
          } else {
            playOneshotVoice(sound_filename_over_15m_);
          }
          if (!waitForStopReasonPlaybackDelay(kDistanceSoundDelay)) {
            return;
          }
          playOneshotVoice(sound_filename_detecting_);
        } else if (reason_name == "ObstacleStop") {
          playOneshotVoice(sound_filename_detecting_route_, is_cut_in_voice);
        } else {
          RCLCPP_INFO(
            this->get_logger(),
            "[stop reasons] no bgm for reason=%s", reason_name.c_str());
        }
        if (!waitForStopReasonPlaybackDelay(kPointSoundCooldown)) {
          return;
        }
      }
    }
  }
}

bool AdSoundManager::waitForStopReasonPlaybackDelay(const std::chrono::seconds & delay)
{
  std::unique_lock<std::mutex> lock(stop_reason_playback_mutex_);
  return !stop_reason_playback_cv_.wait_for(
    lock, delay, [this]() {return stop_reason_playback_cancel_requested_;});
}

void AdSoundManager::stopStopReasonPlayback()
{
  {
    std::lock_guard<std::mutex> lock(stop_reason_playback_mutex_);
    stop_reason_playback_cancel_requested_ = true;
  }
  stop_reason_playback_cv_.notify_all();

  if (stop_reason_playback_thread_.joinable()) {
    stop_reason_playback_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(stop_reason_playback_mutex_);
    stop_reason_playback_cancel_requested_ = false;
  }
}

AdSoundManager::PreSoundType AdSoundManager::checkPreSoundType(void)
{
  if ( (pre_sound_filename_ == sound_filename_turn_left_) ||
    (pre_sound_filename_ == sound_filename_turn_right_) )
  {
    return PreSoundType::TURN_LEFTRIGHT_SOUND;
  } else if ( (pre_sound_filename_ == sound_filename_detecting_) ||
    (pre_sound_filename_ == sound_filename_detecting_route_) ||
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
  if (cur_service_layer_state_ !=
    autoware_state_machine_msgs::msg::StateMachine::STATE_STOP_DUETO_APPROACHING_OBSTACLE)
  {
    stopStopReasonPlayback();
    processed_stop_reasons_ = nullptr;
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
      if (last_stop_reasons_ == nullptr) {
        break;
      }

      requestStopReasonRelativePositionSounds(last_stop_reasons_);
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
      playLoopVoice(sound_filename_turn_left_, is_cut_in_voice);
      break;
    case autoware_state_machine_msgs::msg::StateMachine::STATE_TURNING_RIGHT:
      if ( (continuity_state_ == true) && (turn_state_ == RIGHT) ) {
        // Loop playback has already been instructed.
        break;
      }
      turn_state_ = RIGHT;
      continuity_state_ = true;
      pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
      playLoopVoice(sound_filename_turn_right_, is_cut_in_voice);
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
          playLoopVoice(sound_filename_turn_left_, is_cut_in_voice);
          break;
        }
        else if (turn_signal_ == tier4_vehicle_msgs::msg::TurnSignal::RIGHT) {
          turn_state_ = RIGHT;
          continuity_state_ = true;
          pub_bgm_cmd_->publish(initAudioCmd(sdc_msg_.CMD_VOLUME, VOLUME_LOW_BGM));
          playLoopVoice(sound_filename_turn_right_, is_cut_in_voice);
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
          playLoopVoice(sound_filename_turn_right_, is_cut_in_voice);
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
          playLoopVoice(sound_filename_turn_left_, is_cut_in_voice);
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
      processed_stop_reasons_ = nullptr;
      playLoopNoBGM(sound_filename_bgm_);
      continuity_state_ = false;
      pub_voice_cmd_->publish(initAudioCmd(sdc_msg_.CMD_STOP));
      pre_sound_filename_ = "";
      break;
    default:
      break;
  }
}

}  // namespace ad_sound_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ad_sound_manager::AdSoundManager)
