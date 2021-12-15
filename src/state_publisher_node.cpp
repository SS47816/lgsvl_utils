/* state_publisher_node.cpp

 * Copyright (C) 2021 SS47816

 * ROS Node for Localizing the Ego Vehicle in LGSVL Simulator using the Ground Truth Pose
 * Publish frame transform between odom frame and baselink frame
 
**/

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <lgsvl_msgs/CanBusData.h>
#include <autoware_msgs/VehicleStatus.h>

namespace lgsvl_utils 
{

class StatePublisherNode
{
 public:
  StatePublisherNode();
  virtual ~StatePublisherNode() {};

 private:
  float steering_limit_;
  std::string odom_frame_;
  std::string baselink_frame_;

  ros::NodeHandle nh;
  tf2_ros::TransformBroadcaster tf2_broadcaster;
  // ros::Publisher pub_speed;
  // ros::Publisher pub_accel;
  // ros::Publisher pub_steer;
  ros::Publisher pub_vehicle_state;
  ros::Subscriber sub_lgsvl_odom;
  ros::Subscriber sub_lgsvl_can_bus;
  
  void odomCallback(const nav_msgs::Odometry::ConstPtr& lgsvl_odom_msg);
  void canBusCallback(const lgsvl_msgs::CanBusData::ConstPtr& lgsvl_can_bus_msg);
};

StatePublisherNode::StatePublisherNode()
{
  ros::NodeHandle private_nh("~");
  
  std::string lgsvl_odom_topic;
  std::string lgsvl_can_bus_topic;
  std::string speed_topic;
  std::string vehicle_state_topic;
  
  ROS_ASSERT(private_nh.getParam("lgsvl_odom_topic", lgsvl_odom_topic));
  ROS_ASSERT(private_nh.getParam("lgsvl_can_bus_topic", lgsvl_can_bus_topic));
  ROS_ASSERT(private_nh.getParam("vehicle_state_topic", vehicle_state_topic));
  ROS_ASSERT(private_nh.getParam("odom_frame", odom_frame_));
  ROS_ASSERT(private_nh.getParam("baselink_frame", baselink_frame_));
  ROS_ASSERT(private_nh.getParam("steering_limit", steering_limit_));

  sub_lgsvl_odom = nh.subscribe(lgsvl_odom_topic, 1, &StatePublisherNode::odomCallback, this);
  sub_lgsvl_can_bus = nh.subscribe(lgsvl_can_bus_topic, 1, &StatePublisherNode::canBusCallback, this);

  pub_vehicle_state = nh.advertise<autoware_msgs::VehicleStatus>(vehicle_state_topic, 1);
}

void StatePublisherNode::odomCallback(const nav_msgs::Odometry::ConstPtr& lgsvl_odom_msg)
{
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = odom_frame_;
  transformStamped.child_frame_id = baselink_frame_;
  transformStamped.transform.translation.x = lgsvl_odom_msg->pose.pose.position.x;
  transformStamped.transform.translation.y = lgsvl_odom_msg->pose.pose.position.y;
  transformStamped.transform.translation.z = lgsvl_odom_msg->pose.pose.position.z;
  transformStamped.transform.rotation = lgsvl_odom_msg->pose.pose.orientation;

  tf2_broadcaster.sendTransform(std::move(transformStamped));
}

void StatePublisherNode::canBusCallback(const lgsvl_msgs::CanBusData::ConstPtr& lgsvl_can_bus_msg)
{
  autoware_msgs::VehicleStatus vehicle_status_msg;
  vehicle_status_msg.header = lgsvl_can_bus_msg->header;
  vehicle_status_msg.header.frame_id = baselink_frame_;
  vehicle_status_msg.drivemode = autoware_msgs::VehicleStatus::MODE_MANUAL;
  vehicle_status_msg.speed = lgsvl_can_bus_msg->speed_mps;
  vehicle_status_msg.drivepedal = lgsvl_can_bus_msg->throttle_pct >= 0.05? 1 : 0;
  vehicle_status_msg.brakepedal = lgsvl_can_bus_msg->brake_pct >= 0.05? 1 : 0;
  vehicle_status_msg.angle = lgsvl_can_bus_msg->steer_pct*steering_limit_;
  // Map Gear Options
  if (lgsvl_can_bus_msg->reverse_gear_active)
  {
    vehicle_status_msg.current_gear.gear = autoware_msgs::Gear::REVERSE;
  }
  else
  {
    vehicle_status_msg.current_gear.gear = autoware_msgs::Gear::DRIVE;
  }
  // Map Lamp Options
  if (lgsvl_can_bus_msg->left_turn_signal_active)
  {
    vehicle_status_msg.lamp = autoware_msgs::VehicleStatus::LAMP_LEFT;
  }
  else if (lgsvl_can_bus_msg->right_turn_signal_active)
  {
    vehicle_status_msg.lamp = autoware_msgs::VehicleStatus::LAMP_RIGHT;
  }
  else if (lgsvl_can_bus_msg->hazard_lights_active)
  {
    vehicle_status_msg.lamp = autoware_msgs::VehicleStatus::LAMP_HAZARD;
  }
  else
  {
    vehicle_status_msg.lamp = 0;
  }
  vehicle_status_msg.light = lgsvl_can_bus_msg->fog_lights_active;

  pub_vehicle_state.publish(std::move(vehicle_status_msg));
}

} // namespace lgsvl_utils

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_publisher_node");
  lgsvl_utils::StatePublisherNode state_publisher_node;
  ros::spin();  // spin the ros node.
  return 0;
}