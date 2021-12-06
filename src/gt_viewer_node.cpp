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
  std::string detections3d_topic = "/simulator/ground_truth/3d_detections";
  
  lgsvl_gt3d_sub = nh.subscribe(detections3d_topic, 1, &GTViwerNode::detections3DCallback, this);


}

void GTViwerNode::detections3DCallback(const lgsvl_msgs::Detection3DArray& lgsvl_bboxes)
{
  jsk_recognition_msgs::BoundingBoxArray jsk_bboxes;
  autoware_msgs::DetectedObjectArray autoware_bboxes;

  for (auto const& detection : lgsvl_bboxes.detections)
  {
    jsk_recognition_msgs::BoundingBox jsk_bbox;


    jsk_bboxes.boxes.emplace_back(jsk_bbox);

    autoware_msgs::DetectedObject autoware_bbox;


    autoware_bboxes.objects.emplace_back(autoware_bbox);
  }

  jsk_bboxes.header = lgsvl_bboxes.header;
  autoware_bboxes.header = lgsvl_bboxes.header;
}

}