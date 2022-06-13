# autonomous_driving_sound_manager

## Overview
Any autonomous driving vehicle have to notify its action to surrounding people.

This node selects both a recorded voice and BGM depending on a state of whole system which combined the Autoware and a vehicle.

This node plays sound in the following order of priority.
1. Alert people around the ego vehicle once before the ego vehicle departs. The ego vehicle will not depart until the alert is complete.
2. Keep alerting obstacles around the ego vehicle.
3. Keep alerting obstacles in the ego vehicle's path.
4. Notify the cargo receiver once that the ego vehicle has arrived.
5. Keep informed that the ego vehicle is avoiding obstacles.
6. Keep informed that the ego vehicle is turning left or right.
7. Keep requesting an departure permit for on-demand delivery.
8. Keep alerting people around the ego vehicle while driving.
9. Notify operator once that the system has booted successfully.
10. Keep informed that the system is being in shut down sequence.

## Input and Output
- input
  - from [autoware.universe](https://github.com/autowarefoundation/autoware.universe)
    - `/awapi/vehicle/get/status` : Vehicle status. Refers to the status of the turn signal.
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `/autoware_state_machine/state` : State of the system.
  - from sound_voice_alarm/[audio_driver](https://github.com/eve-autonomy/audio_driver)
    - `/sound_voice_alarm/audio_res` : Notification that voice alarm playback is complete.
- output
  - to [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `/autoware_state_machine/state_sound_done` : Notification that sound playback is complete.
  - to sound_voice_alarm/[audio_driver](https://github.com/eve-autonomy/audio_driver)
    - `/sound_voice_alarm/audio_cmd` : Voice alarm playback request.
  - to sound_bgm/[audio_driver](https://github.com/eve-autonomy/audio_driver)
    - `/sound_bgm/audio_cmd` : BGM playback requst.
## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/eve-autonomy/ad_sound_manager/docs/node_graph.pu)

## Parameter discription

<table>
  <thead>
    <tr>
      <th scope="col">Name</th>
      <th scope="col">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
	    <td>sound_filename_avoid</td>
	    <td>File name of the voice alert for obstacle avoidance.</td>
    <tr>
	    <td>sound_filename_start</td>
	    <td>File name of the voice alert for engaging the ego vehicle.</td>
    </tr>
    <tr>
	    <td>sound_filename_left</td>
	    <td>File name of the voice alert for turning left.</td>
    </tr>
    <tr>
	    <td>sound_filename_right</td>
	    <td>File name of the voice alert for turning right.</td>
    </tr>
    <tr>
	    <td>sound_filename_bgm</td>
	    <td>BGM file name as the driving warning sound.</td>
    </tr>
    <tr>
	    <td>sound_filename_obstacle</td>
	    <td>Warning sound file name for obstacles in the ego vehicle's path.</td>
    </tr>
    <tr>
	    <td>sound_filename_wakeup</td>
	    <td>Sound file name for system startup notification.</td>
    </tr>
    <tr>
	    <td>sound_filename_leave</td>
	    <td>Warning sound file name for obstacles around the ego vehicle.</td>
    </tr>
    <tr>
	    <td>sound_filename_arrival</td>
	    <td>Sound file name to notify the arrival of the ego vehicle.</td>
    </tr>
    <tr>
	    <td>sound_filename_call</td>
	    <td>Sound file name asking for permission to engage the ego vehicle for on-demand delivery.</td>
    </tr>
  </tbody>
</table>

The specific values for these parameters are defined in the ad_sound package.

If you want to use different sound, fork the [ad_sound.default](https://github.com/eve-autonomy/ad_sound.default) repository, create a new repository.
