# lgsvl-utils

ROS Helper Nodes for utilising LGSVL Simulator in Autonomous Vehicles Development

[![CodeFactor](https://www.codefactor.io/repository/github/ss47816/lgsvl_utils/badge)](https://www.codefactor.io/repository/github/ss47816/lgsvl_utils)
![Code Grade](https://api.codiga.io/project/30669/status/svg)
![Code Quality Score](https://api.codiga.io/project/30669/score/svg)
![GitHub Repo stars](https://img.shields.io/github/stars/ss47816/lgsvl_utils?color=FFE333)
![GitHub Repo forks](https://img.shields.io/github/forks/ss47816/lgsvl_utils?color=FFE333)

![Ubuntu](https://img.shields.io/badge/OS-Ubuntu-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS](https://img.shields.io/badge/Tools-ROS-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)

![cover_image](media/demo.gif)

## Updates
* [19 Oct 2022] Major refactor of the repo, included minimal required [autoware_ai](https://github.com/autowarefoundation) components so now you can use this repo **out-of-the box**.
## Features
* **Ground Truth Objects**: Convert `lgsvl` 3D ground truth objects to `autoware` and `jsk_recognition` messages and visualize them in Rviz
* **Vehicle Status Publishing**: The ego vehicle's Can Bus data are extracted from the `lgsvl` simulator and published to ROS
* **Map Format Supports**: Support `pointcloud map`, `lanelet2 map`, `vector map` formats
* **Global Planning**: Provide Global Planning in `lanelet2 map` format (Issue discovered, fixing...)
* **Joystick Control**: Drive in the Simulator with a joystick (currently support `Xbox`, `Logitech F710` joysticks, and control settings like `Forza Horizon`, `Japan Hand`, `USA Hand`)
* **Vehicle State Control**: Control the Ego Vehicle's State with `lgsvl_msgs::VehicleStateData` message (including control of all the lights, blinkers, wipers, etc)
* **Vehicle Motion Control**: Control the Ego Vehicle's Motion with `lgsvl_msgs::VehicleControlData` message (with Reversing Enabled)
* **Vehicle Model**: Display the ego vehicle model in Rviz

**TODOs**
* feat: 2D Ground Truth Objects
* feat: Support for Other Sensors
* feat: Relocalization?

**Known Issues**
* The official BorregasAve lanelet2 map file contains minor connection errors (not going to fix)

## Dependencies
* System Requirements:
  * Ubuntu 18.04/20.04
  * ROS Melodic/Noetic
  * C++11 above
  * CMake: 3.0.2 above
* This package is self-contained, only dependens on standard ROS pkgs:
  * jsk_recognition_msgs
  * autoware_msgs
  * lgsvl_msgs
  * visualization_msgs
  * geometry_msgs
  * sensor_msgs
  * nav_msgs
  * std_msgs
  * roscpp
  * rospy
  * tf2_ros
  * tf2_eigen
  * tf2_geometry_msgs
  * tf
  * joy
  * cv_bridge
  * image_transport
  * rosbridge_server

## Installation
To use this package, you will need to create a `catkin_ws` first. Details please see the [ROS official tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
```bash
# clone the repo into your catkin workspace (assuming ~/catkin_ws here)
cd ~/catkin_ws/src
git clone https://github.com/SS47816/lgsvl_utils.git
cd ..

# install dependencies
rosdep install --from-paths src --ignore-src -r -y

# build
catkin_make
# source 
source devel/setup.bash
```

## Usage
### Example BorregasAve Map in LGSVL
**Step 1**: Download LGSVL Simulator and register an account from [here](https://www.svlsimulator.com/)

**Step 2**: Download the LGSVL maps from the [official repo](https://github.com/lgsvl/autoware-data)
```bash
# create a directory called `shared_dir` under HOME directory
cd
mkdir shared_dir
cd ~/shared_dir
# clone the autoware map data into this directory
git clone https://github.com/lgsvl/autoware-data.git
```

Replace all the contents in `~/shared_dir/autoware-data/BorregasAve/` with the contents in `lgsvl_utils/lgsvl_utils/lgsvl_assets/BorregasAve/`

**Step 3**: Create a new simulation on the LGSVL simulator web client.
1. On the `General` page, fill in some information and select your local cluster.
2. On the `Test Case` page, select the `BorregasAve` Map, `UT Lexus` Vehicle and upload the sensor configuration provided in `lgsvl_utils/lgsvl_assets/sensor_config/[Custom] Autoware AI.json`
3. On the `Autopilot` page, select `Autoware.AI v1.14` and use the default `localhost:9090`

**Step 4**: Start the simulation

**Step 5**: Launch the nodes in this `lgsvl_utils` pkg using the `lgsvl_utils/launch/lgsvl_borregas.launch`
```bash
# launch the all the nodes on the example BorregasAve Map
roslaunch lgsvl_utils borregas.launch 
```

![demo_image](media/demo.png)

**Step 6**: You may now use a [Xbox](https://www.xbox.com/en-SG/accessories/controllers/xbox-wireless-controller) or [Logitech F710](https://www.logitechg.com/en-us/products/gamepads/f710-wireless-gamepad.html) joystick to control the ego vehicle. You may change the joystick settings in `launch/common.launch` (`joy_type` and `control_setting`) or customize the button mapping accroding to your own preference in `src/joystick_node.cpp`.

**Step 7**: The mode of the ego vehicle is fully controlled by the joystick, for example: `A` -> Autonomous Mode, `B` -> Brake Mode, `X` -> Manual Mode, `Y` -> Manual Reverse Mode
(**Note:** For safety considerations, the Autonomous Mode (`A`) can only be started when the vehicle is in Brake Mode (`B`).)

## Contribution
You are welcome contributing to the package by opening a pull-request

We are following: 
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), 
[C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main), 
and [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

## License
The `lgsvl_utils` is released under the [MIT License](https://github.com/SS47816/lgsvl_utils/blob/main/LICENSE)

The included `autoware_ai` components follow their own [Apache License 2.0](https://github.com/autowarefoundation/autoware_ai_common/blob/master/LICENSE)