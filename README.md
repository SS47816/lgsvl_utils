# lgsvl-utils

ROS Helper Nodes for visualising and utilising LGSVL simulator

## Current Functions
* **Visualizing Objects**: Converte `lgsvl` 3D Ground Truth Objects to `autoware` and `jsk` messages
* **Vehicle Localization**: Recieve `lgsvl` Ego Vehicle Ground Truth Pose and publish frame transform between `map` and `baselink`
* **Map Format Supports**: `pointcloud map`, `lanelet2 map`, `vector map`
* **Global Planners**:  `lanelet2 map`, `vector map` (fixing issues)
* **Vehicle Model**:  Display Vehicle model in Rviz

**TODOs**
* Relocalization?
* Vehicle Control (Enable Reversing)
* Joystick Control (Xbox, Logitech F710)

* 2D Ground Truth Objects
* Support for Other Sensors

**Known Issues**
* Map elements idle
* BorregasAve lanelet2 map wrong connections

## Dependencies
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