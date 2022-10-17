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
* feat: Packing functions into this pkg
* feat: 2D Ground Truth Objects
* feat: Support for Other Sensors
* feat: Relocalization?

**Known Issues**
* BorregasAve lanelet2 map contains minor connection errors
* PCD Map stays idle during driving (suspected to be the publishing rate issue)

## Dependencies
* lanelet2
* lgsvl-msgs
* autoware-msgs
* jsk-recognition-msgs

## Installation
```bash
# clone the repo
cd catkin_ws/src
git clone https://github.com/SS47816/lgsvl_utils.git

# install dependencies & build 
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Usage
### Example BorregasAve Map in LGSVL
**Step 1**: Download LGSVL Simulator and register an account

**Step 2**: Select a the `UT Lexus` Vehicle and upload the sensor configuration provided in `lgsvl_utils/lgsvl_assets/sensor_config/[Custom] Autoware AI.json`

**Step 3**: Select the BorregasAve Map and start the simulation

**Step 4**: Launch the nodes in this `lgsvl_utils` pkg using the `lgsvl_utils/launch/lgsvl_borregas.launch`
```bash
# launch the all the nodes on the example BorregasAve Map
roslaunch lgsvl_utils borregas.launch 
```

![demo_image](media/demo.png)

**Step 5**: You may now use a [Xbox](https://www.xbox.com/en-SG/accessories/controllers/xbox-wireless-controller) or [Logitech F710](https://www.logitechg.com/en-us/products/gamepads/f710-wireless-gamepad.html) joystick to control the ego vehicle. You may change the joystick settings in `launch/common.launch` (`joy_type` and `control_setting`) or customize the button mapping accroding to your own preference in `src/joystick_node.cpp`.

## Contribution
You are welcome contributing to the package by opening a pull-request

We are following: 
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), 
[C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main), 
and [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

## License
MIT License