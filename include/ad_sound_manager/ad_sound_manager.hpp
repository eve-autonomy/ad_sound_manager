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

#include "audio_driver_msgs/msg/sound_driver_ctrl.hpp"
#include "audio_driver_msgs/msg/sound_driver_res.hpp"
#include "autoware_state_machine_msgs/msg/state_machine.hpp"
#include "autoware_state_machine_msgs/msg/state_sound_done.hpp"
#include "tier4_api_msgs/msg/awapi_vehicle_status.hpp"
#include "tier4_vehicle_msgs/msg/turn_signal.hpp"

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
  ~AdSoundManager();

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

  rclcpp::Publisher<audio_driver_msgs::msg::SoundDriverCtrl>::SharedPtr pub_bgm_cmd_, pub_voice_cmd_;
  rclcpp::Publisher<autoware_state_machine_msgs::msg::StateSoundDone>::SharedPtr pub_sound_done_;
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateMachine>::SharedPtr sub_state_;
  rclcpp::Subscription<audio_driver_msgs::msg::SoundDriverRes>::SharedPtr sub_bgm_res_, sub_voice_res_;
  rclcpp::Subscription<tier4_api_msgs::msg::AwapiVehicleStatus>::SharedPtr
    sub_awapi_vehicle_state_;

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

  // Store the status of one-time playback waiting for a response.
  int one_play_state_;

  // Store the latest status receive from autoware_state_machine.
  uint16_t cur_service_layer_state_;
  uint16_t prev_service_layer_state_;
  uint8_t cur_control_layer_state_;
  uint8_t prev_control_layer_state_;

  void makeFullPathWithFileCheck(std::string & file_path);
  void callbackVoiceRes(const audio_driver_msgs::msg::SoundDriverRes::ConstSharedPtr msg);
  void callbackAutowareStateMachine(
    const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg);
  void callbackAwapiVehicleState(
    const tier4_api_msgs::msg::AwapiVehicleStatus::ConstSharedPtr msg);

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
  std::string pre_sound_filename_ = "";

  std::string sound_directory_path_ = "";
};

}  // namespace ad_sound_manager
#endif  // AD_SOUND_MANAGER__AD_SOUND_MANAGER_HPP_
