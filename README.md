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

## Input and Output
- input
  - from [autoware.universe](https://github.com/autowarefoundation/autoware.universe)
    - `/awapi/vehicle/get/status` : Vehicle status. Refers to the status of the turn signal.
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `/autoware_state_machine/state` : State of the system.
  - from sound_voice_alarm/[audio_driver](https://github.com/eve-autonomy/audio_driver)
    - `/sound_voice_alarm/audio_res` : Acknowledgement that voice alarm playback is complete.
- output
  - to [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `/autoware_state_machine/state_sound_done` : Acknowledgement that sound playback is complete.
  - to sound_voice_alarm/[audio_driver](https://github.com/eve-autonomy/audio_driver)
    - `/sound_voice_alarm/audio_cmd` : Voice alarm playback request.
  - to sound_bgm/[audio_driver](https://github.com/eve-autonomy/audio_driver)
    - `/sound_bgm/audio_cmd` : BGM playback request.
## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/eve-autonomy/ad_sound_manager/main/docs/node_graph.pu)

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
