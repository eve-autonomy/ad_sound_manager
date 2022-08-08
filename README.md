# Autonomous Driving: Sound Manager

## Overview
Every autonomous driving vehicle notifies its action to surrounding people.

This node selects both a recorded voice and BGM depending on a state of whole system which combined the Autoware and a vehicle. <br>
This node plays sound in the following order of priority.
1. Alert people around the ego vehicle by `voice alarm` once before the ego vehicle departs.<br> The ego vehicle will not depart until the alert is complete.
1. Keep alerting obstacles around the ego vehicle by `voice alarm`.
1. Keep alerting obstacles in the ego vehicle's path by `voice alarm`.
1. Notify the cargo receiver by `voice alarm` once that the ego vehicle has arrived.
1. Keep informed by `voice alarm` that the ego vehicle is avoiding obstacles.
1. Keep informed by `voice alarm` that the ego vehicle is turning left or right.
1. Keep requesting an departure permit by `voice alarm` for on-demand delivery.
1. Keep alerting people around the ego vehicle by `bgm` while driving.
1. Notify operator by `voice alarm` once that the system has booted successfully.
1. Keep informed by `voice alarm` that the system is being in shut down sequence.

Audio playback interruption function due to idling.
Even when the car is not running, the background music keeps playing quietly to prevent idling.

## Input and Output
- input
  - from [autoware.universe](https://github.com/autowarefoundation/autoware.universe)
    - `/awapi/vehicle/get/status` \[[tier4_api_msgs/msg/awapi_vehicle_status][VehicleStatus]\]:<br>Vehicle status. Refers to the status of the turn signal.
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `/autoware_state_machine/state` \[[autoware_state_machine_msgs/msg/StateMachine][AWState]\]:<br>State of the system.
  - from sound_voice_alarm/[audio_driver](https://github.com/eve-autonomy/audio_driver)
    - `/sound_voice_alarm/audio_res` \[[audio_driver_msgs/msg/SoundDriverRes][SDRes]\]:<br>Acknowledgement that voice alarm playback is complete.
- output
  - to [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `/autoware_state_machine/state_sound_done` \[[autoware_state_machine_msgs/msg/StateSoundDone][SoundDone]\]:<br>Acknowledgement that sound playback is complete.
  - to sound_voice_alarm/[audio_driver](https://github.com/eve-autonomy/audio_driver)
    - `/sound_voice_alarm/audio_cmd` \[[audio_driver_msgs/msg/SoundDriverCtrl][SDCtrl]\]:<br>Voice alarm playback request.
  - to sound_bgm/[audio_driver](https://github.com/eve-autonomy/audio_driver)
    - `/sound_bgm/audio_cmd` \[[audio_driver_msgs/msg/SoundDriverCtrl][SDCtrl]\]:<br>BGM playback request.

[VehicleStatus]: https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_api_msgs/msg/AwapiVehicleStatus.msg
[AWState]: https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/StateMachine.msg
[SoundDone]: https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/StateSoundDone.msg
[SDRes]: https://github.com/eve-autonomy/audio_driver_msgs/blob/main/msg/SoundDriverRes.msg
[SDCtrl]: https://github.com/eve-autonomy/audio_driver_msgs/blob/main/msg/SoundDriverCtrl.msg

## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/ad_sound_manager/main/docs/node_graph.pu)

## Launch arguments
|Name|Description|
|:---|:----------|
|lang|Switch a set of sound by directory name. See [ad_sound.default](https://github.com/eve-autonomy/ad_sound.default#extensibility-of-this-package) for details.|

## Parameter description

|Name|Description|
|:---|:----------|
|sound_filename_avoid|File name of a voice alert for obstacle avoidance.|
|sound_filename_start|File name of a voice alert for engaging the ego vehicle.|
|sound_filename_left |File name of a voice alert for turning left.|
|sound_filename_right|File name of a voice alert for turning right.|
|sound_filename_bgm|BGM file name as the driving warning sound.|
|sound_filename_obstacle|Warning sound file name for obstacles in the ego vehicle's path.|
|sound_filename_wakeup|Sound file name for system startup notification.|
|sound_filename_leave|Warning sound file name for obstacles around the ego vehicle.|
|sound_filename_arrival|Sound file name to notify the arrival of the ego vehicle.|
|sound_filename_call|Sound file name asking for permission to engage the ego vehicle for on-demand delivery.|

The specific values for these parameters are defined in the ad_sound package.

If you want to use different sound, fork the [ad_sound.default](https://github.com/eve-autonomy/ad_sound.default) repository, create a new repository.
