# lgsvl-utils

ROS Helper Nodes for visualising and utilising LGSVL simulator

## Current Functions
* **Visualizing Objects**: Converting `lgsvl` 3D/2D Ground Truth Objects to `autoware` and `jsk` messages
* **Vehicle Localization**: Recieving `lgsvl` Ego Vehicle Ground Truth Pose and publishing frame transform between `map` and `baselink`

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
```

## Contribution

## License
MIT License