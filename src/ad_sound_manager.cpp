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

  sub_voice_res_ = this->create_subscription<audio_driver_msgs::msg::SoundDriverRes>(
    "/sound_voice_alarm/audio_res",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&AdSoundManager::callbackVoiceRes, this, std::placeholders::_1)
  );

  pub_voice_cmd_ = this->create_publisher<audio_driver_msgs::msg::SoundDriverCtrl>(
    "/sound_voice_alarm/audio_cmd", rclcpp::QoS{5}.transient_local());

  pub_bgm_cmd_ = this->create_publisher<audio_driver_msgs::msg::SoundDriverCtrl>(
    "/sound_bgm/audio_cmd", rclcpp::QoS{5}.transient_local());

  pub_sound_done_ =
    this->create_publisher<autoware_state_machine_msgs::msg::StateSoundDone>(
    "/autoware_state_machine/state_sound_done", rclcpp::QoS{3}.transient_local());

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
}

void AdSoundManager::callbackAwapiVehicleState(
  const tier4_api_msgs::msg::AwapiVehicleStatus::ConstSharedPtr msg)
{
  if (turn_signal_ != msg->turn_signal) {
    turn_signal_ = msg->turn_signal;
    changeSoundState(cur_service_layer_state_, cur_control_layer_state_, true);
  }
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

}  // namespace ad_sound_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ad_sound_manager::AdSoundManager)
