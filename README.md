# lgsvl-utils

ROS Helper Nodes for utilising LGSVL Simulator in Autonomous Vehicles Development

![cover_image](media/demo.png)

<video controls="controls">
  <source type="video/mp4" src="media/demo.mp4"></source>
  <p>Your browser does not support the video element.</p>
</video>

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
* feat: Control vehicle states using joystick
* fix: Global Plan to `nav_msgs::Path` centerline coordinates
* feat: Packing functions into this pkg
* feat: 2D Ground Truth Objects
* feat: Support for Other Sensors
* feat: Relocalization?

**Known Issues**
* Global Plan to `nav_msgs::Path` centerline coordinates wrong
* BorregasAve lanelet2 map connection errors
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
**Step 1**: download 

```bash
# launch the example BorregasAve Map
roslaunch lgsvl_utils lgsvl_borregas.launch 
```


## Contribution

## License
MIT License