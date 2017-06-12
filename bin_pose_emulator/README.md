# bin_pose_emulator

Generator of random bin poses. It provides /bin_pose service which returns a combination of 3 bin picking points in predefined bin:

* **Grasp Point**
* **Approach Point**
* **Deapproach point**
The Grasp point is the goal pose in predefined Virtual Bin. The Approach point is located at the normal vector given by Grasp Point orientation and the Deapproach point is located right above the Grasp point.

Following [video](https://youtu.be/l4nY1mkcvU8) demonstrates the capabilities of bin_pose_emulator in combination with ABB IRB1200 and MoveIt.

### Installation
This package has been released as ROS Kinetic binaries, installation via apt-get is possible:

```
sudo apt-get install ros-kinetic-binpicking-utils
```

### Usage

```
roslaunch bin_pose_emulator bin_pose_emulator
```
Call ROS service with an Empty request:
```
rosservice call /bin_pose
```

Service response:
```
grasp_pose: 
  position: 
    x: 0.582990474831
    y: 0.132920391803
    z: 0.110564293991
  orientation: 
    x: 0.113430335549
    y: 0.71905280478
    z: -0.119797227723
    w: 0.675089066083
approach_pose: 
  position: 
    x: 0.589268137291
    y: 0.132782671333
    z: 0.210366960715
  orientation: 
    x: 0.113430335549
    y: 0.71905280478
    z: -0.119797227723
    w: 0.675089066083
deapproach_pose: 
  position: 
    x: 0.582990474831
    y: 0.132920391803
    z: 0.260564299952
  orientation: 
    x: 0.113430335549
    y: 0.71905280478
    z: -0.119797227723
    w: 0.675089066083
```

### Visualization

In order to simplify configuration and usage of this emulator, basic visualization is available - **bin_pose_emulator** publishes two  messages to **bin_pose_visualization** topic. Use **Marker** view in RViz to visualize the pose of the virtual bin and the location of the current grasp point. Both Markers are published with *base_link* as a reference frame. 

<img src="http://www.smartroboticsys.eu/wp-content/uploads/2016/12/bin_pose_emulator.jpg" width="750">

### Config

Virtual Bin is defined by **bin_center_** and **bin_size_** parameters, while allowed orientation is set by roll, pitch and yaw range. By default we assume that **tool0 link** of robot is aligned with vertical axis, pointing to the ground - RPY is [0,90,0]. **roll_range**, **pitch_range** and **yaw_range** extend default pose orientation into allowed grasp cone. 
**Approach distance** defines how far in direction of the normal vector given by the Grasp Point orientation is the Approach point. The **deapproach_height** configures height of vertical movement when moving away from the Grasp Point. Example config file is located at the onfig folder. Change launch file parameter **filepath** to adopt launch to your custom config file.

Example Yaml config file: 
```
bin_center_x: 0.5
bin_center_y: 0
bin_center_z: 0.1

bin_size_x: 0.2
bin_size_y: 0.5
bin_size_z: 0.1

roll_default: 0
pitch_default: 3.14
yaw_default: 0

roll_range: 0.707
pitch_range: 0.707
yaw_range: 0.707

approach_distance: 0.1
deapproach_height: 0.15
```

