/* localizer_node.cpp

 * Copyright (C) 2021 SS47816

 * ROS Node for Localizing the Ego Vehicle in LGSVL Simulator using the Ground Truth Pose
 * Publish frame transform between odom and baselink
 
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

namespace lgsvl_utils 
{

class LocalizerNode
{
 public:
  LocalizerNode();
  virtual ~LocalizerNode() {};

 private:
  ros::NodeHandle nh;
  ros::Subscriber sub_lgsvl_odom;
  tf2_ros::TransformBroadcaster tf2_broadcaster;

  std::string odom_frame_;
  std::string baselink_frame_;
  
  void odomCallback(const nav_msgs::Odometry::ConstPtr& lgsvl_odom_msg);
};

LocalizerNode::LocalizerNode()
{
  ros::NodeHandle private_nh("~");
  
  std::string lgsvl_odom_topic;
  
  ROS_ASSERT(private_nh.getParam("lgsvl_odom_topic", lgsvl_odom_topic));
  ROS_ASSERT(private_nh.getParam("odom_frame", odom_frame_));
  ROS_ASSERT(private_nh.getParam("baselink_frame", baselink_frame_));

  sub_lgsvl_odom = nh.subscribe(lgsvl_odom_topic, 1, &LocalizerNode::odomCallback, this);
}

void LocalizerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& lgsvl_odom_msg)
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

} // namespace lgsvl_utils

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localizer_node");
  lgsvl_utils::LocalizerNode localizer_node;
  ros::spin();  // spin the ros node.
  return 0;
}