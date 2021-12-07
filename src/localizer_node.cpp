/* localization.cpp

  Copyright (C) 2021 SS47816 & Advanced Robotics Center, National University of Singapore

  Localizing Ego Vehicle in LGSVL Simulator using Ground Truth Pose
*/

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
  ros::Subscriber sub_ll2_map_viz;
  ros::Publisher pub_ll2_map_viz;
  tf2_ros::TransformBroadcaster tf2_broadcaster;

  std::string world_frame_;
  std::string map_frame_;
  std::string baselink_frame_;
  
  void odomCallback(const nav_msgs::Odometry::ConstPtr& lgsvl_odom_msg);
  void ll2MapCallback(const visualization_msgs::MarkerArray::ConstPtr& ll2_map_viz_msg);
};

LocalizerNode::LocalizerNode()
{
  ros::NodeHandle private_nh("~");
  
  std::string lgsvl_odom_topic;
  std::string input_ll2_map_viz_topic;
  std::string output_ll2_map_viz_topic;
  
  ROS_ASSERT(private_nh.getParam("lgsvl_odom_topic", lgsvl_odom_topic));
  ROS_ASSERT(private_nh.getParam("input_ll2_map_viz_topic", input_ll2_map_viz_topic));
  ROS_ASSERT(private_nh.getParam("output_ll2_map_viz_topic", output_ll2_map_viz_topic));
  ROS_ASSERT(private_nh.getParam("world_frame", world_frame_));
  ROS_ASSERT(private_nh.getParam("map_frame", map_frame_));
  ROS_ASSERT(private_nh.getParam("baselink_frame", baselink_frame_));

  sub_lgsvl_odom = nh.subscribe(lgsvl_odom_topic, 1, &LocalizerNode::odomCallback, this);
  sub_ll2_map_viz = nh.subscribe(input_ll2_map_viz_topic, 1, &LocalizerNode::ll2MapCallback, this);
  pub_ll2_map_viz = nh.advertise<visualization_msgs::MarkerArray>(output_ll2_map_viz_topic, 1);
}

void LocalizerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& lgsvl_odom_msg)
{
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = map_frame_;
  transformStamped.child_frame_id = baselink_frame_;
  transformStamped.transform.translation.x = lgsvl_odom_msg->pose.pose.position.x;
  transformStamped.transform.translation.y = lgsvl_odom_msg->pose.pose.position.y;
  transformStamped.transform.translation.z = lgsvl_odom_msg->pose.pose.position.z;
  transformStamped.transform.rotation = lgsvl_odom_msg->pose.pose.orientation;

  tf2_broadcaster.sendTransform(std::move(transformStamped));
}

void LocalizerNode::ll2MapCallback(const visualization_msgs::MarkerArray::ConstPtr& ll2_map_viz_msg)
{
  visualization_msgs::MarkerArray ll2_world_viz_msg;
  ROS_WARN("############# There are %d markers in total", int(ll2_map_viz_msg->markers.size()));
  for (auto &marker : ll2_map_viz_msg->markers)
  {
    visualization_msgs::Marker new_marker = marker;
    new_marker.header.frame_id = world_frame_;
    ll2_world_viz_msg.markers.emplace_back(std::move(new_marker));
  }
  pub_ll2_map_viz.publish(ll2_world_viz_msg);
  ROS_WARN("############# I Published %d markers in total", int(ll2_world_viz_msg.markers.size()));
}

} // namespace lgsvl_utils

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localizer_node");
  lgsvl_utils::LocalizerNode localizer_node;
  ros::spin();  // spin the ros node.
  return 0;
}