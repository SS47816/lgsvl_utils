# pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <lgsvl_msgs/Detection3D.h>
#include <lgsvl_msgs/Detection3DArray.h>
#include <lgsvl_msgs/BoundingBox3D.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <autoware_msgs/DetectedObjectArray.h>

namespace lgsvl_utils 
{

class GTViwerNode
{
 public:
  GTViwerNode();
  virtual ~GTViwerNode() {};

 private:
  ros::NodeHandle nh;
  ros::Subscriber lgsvl_gt3d_sub;
  ros::Publisher jsk_bboxes_pub;
  ros::Publisher autoware_bboxes_pub;

  void detections3DCallback(const lgsvl_msgs::Detection3DArray& lgsvl_bboxes);
};

GTViwerNode::GTViwerNode()
{
  ros::NodeHandle private_nh("~");
  
  std::string lgsvl_gt3d_topic;
  std::string jsk_bboxes_topic;
  std::string autoware_bboxes_topic;
  
  ROS_ASSERT(private_nh.getParam("lgsvl_gt3d_topic", lgsvl_gt3d_topic));
  ROS_ASSERT(private_nh.getParam("jsk_bboxes_topic", jsk_bboxes_topic));
  ROS_ASSERT(private_nh.getParam("autoware_bboxes_topic", autoware_bboxes_topic));

  lgsvl_gt3d_sub = nh.subscribe(lgsvl_gt3d_topic, 1, &GTViwerNode::detections3DCallback, this);
  jsk_bboxes_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(jsk_bboxes_topic, 1);
  autoware_bboxes_pub = nh.advertise<autoware_msgs::DetectedObjectArray>(autoware_bboxes_topic, 1);
}

void GTViwerNode::detections3DCallback(const lgsvl_msgs::Detection3DArray& lgsvl_bboxes)
{
  jsk_recognition_msgs::BoundingBoxArray jsk_bboxes;
  autoware_msgs::DetectedObjectArray autoware_bboxes;
  jsk_bboxes.header = lgsvl_bboxes.header;
  autoware_bboxes.header = lgsvl_bboxes.header;

  for (auto const& lgsvl_bbox : lgsvl_bboxes.detections)
  {
    jsk_recognition_msgs::BoundingBox jsk_bbox;
    jsk_bbox.header = lgsvl_bbox.header;
    jsk_bbox.pose = lgsvl_bbox.bbox.position;
    jsk_bbox.dimensions = lgsvl_bbox.bbox.size;
    jsk_bbox.value = lgsvl_bbox.score;
    if (lgsvl_bbox.label == "Pedestrian")
    {
      jsk_bbox.label = 2;
    }
    else if (lgsvl_bbox.label == "Bicyclist")
    {
      jsk_bbox.label = 1;
    }
    else
    {
      jsk_bbox.label = 0;
    }

    jsk_bboxes.boxes.emplace_back(jsk_bbox);

    autoware_msgs::DetectedObject autoware_bbox;
    autoware_bbox.header = lgsvl_bbox.header;
    autoware_bbox.id = lgsvl_bbox.id;
    autoware_bbox.label = lgsvl_bbox.label;
    autoware_bbox.score = lgsvl_bbox.score;
    autoware_bbox.pose = lgsvl_bbox.bbox.position;
    autoware_bbox.dimensions = lgsvl_bbox.bbox.size;
    autoware_bbox.velocity = lgsvl_bbox.velocity;
    
    autoware_bboxes.objects.emplace_back(autoware_bbox);
  }

  jsk_bboxes_pub.publish(jsk_bboxes);
  autoware_bboxes_pub.publish(autoware_bboxes);
}

} // namespace lgsvl_utils

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gt_viewer_node");
  lgsvl_utils::GTViwerNode gt_viewer_node;
  ros::spin();  // spin the ros node.
  return 0;
}