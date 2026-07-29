# Autonomous Driving: Sound Manager

## Overview
`ad_sound_manager` selects voice prompts and background music based on the combined Autoware and vehicle state.

## Sound behavior
Playback priority is:

1. Notify people around the ego vehicle before departure. The vehicle will not depart until the alert finishes.
1. Alert on obstacles around the ego vehicle.
1. Alert on obstacles in the ego vehicle's path.
1. Notify the cargo receiver when the ego vehicle arrives.
1. Notify obstacle avoidance.
1. Notify left/right turns.
1. Request a departure permit for on-demand delivery.
1. Play background music while driving to alert nearby people.
1. Notify the operator when the system boots.
1. Notify the operator during shutdown.

## Anti-idling for audio devices
The node keeps audio devices awake by playing very low-volume background music when no other audio is requested.

Without this, some devices enter an idle state and clip the first sound after playback resumes. Keeping a silent-ish BGM stream active avoids that interruption.

## Interfaces

### Subscriptions

| Topic | Message | Description |
|:---|:---|:---|
| `/awapi/vehicle/get/status` | [`AwapiVehicleStatus`][AwapiVehicleStatus] | Vehicle status, including turn signal state. |
| `/autoware_state_machine/state` | [`autoware_state_machine_msgs/msg/StateMachine`][AWState] | Current system state. |
| `/sound_voice_alarm/audio_res` | [`audio_driver_msgs/msg/SoundDriverRes`][SDRes] | Voice-alarm playback completion notice. |

### Publications

| Topic | Message | Description |
|:---|:---|:---|
| `/autoware_state_machine/state_sound_done` | [`autoware_state_machine_msgs/msg/StateSoundDone`][SoundDone] | Sound playback completion notice. |
| `/sound_voice_alarm/audio_cmd` | [`audio_driver_msgs/msg/SoundDriverCtrl`][SDCtrl] | Voice-alarm playback request. |
| `/sound_bgm/audio_cmd` | [`audio_driver_msgs/msg/SoundDriverCtrl`][SDCtrl] | BGM playback request. |

[AwapiVehicleStatus]: https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_api_msgs/msg/AwapiVehicleStatus.msg
[AWState]: https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/StateMachine.msg
[SoundDone]: https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/StateSoundDone.msg
[SDRes]: https://github.com/eve-autonomy/audio_driver_msgs/blob/main/msg/SoundDriverRes.msg
[SDCtrl]: https://github.com/eve-autonomy/audio_driver_msgs/blob/main/msg/SoundDriverCtrl.msg

## Node graph
![node graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/ad_sound_manager/main/docs/node_graph.pu)

## Launch arguments

| Name | Description |
|:---|:---|
| `lang` | Selects a sound set by directory name. See [ad_sound.default](https://github.com/eve-autonomy/ad_sound.default#extensibility-of-this-package) for details. |

## Parameters

| Name | Description |
|:---|:---|
| `sound_filename_avoid` | Voice alert file for obstacle avoidance. |
| `sound_filename_start` | Voice alert file for engaging the ego vehicle. |
| `sound_filename_left` | Voice alert file for turning left. |
| `sound_filename_right` | Voice alert file for turning right. |
| `sound_filename_bgm` | BGM file used as the driving warning sound. |
| `sound_filename_obstacle` | Warning sound file for obstacles in the ego vehicle's path. |
| `sound_filename_wakeup` | Sound file for the system startup notification. |
| `sound_filename_leave` | Warning sound file for obstacles around the ego vehicle. |
| `sound_filename_arrival` | Sound file that notifies arrival. |
| `sound_filename_call` | Sound file that requests permission for on-demand delivery departure. |

The actual values for these parameters are defined in the ad_sound package.

To use different sounds, fork [ad_sound.default](https://github.com/eve-autonomy/ad_sound.default) and create a new repository.
