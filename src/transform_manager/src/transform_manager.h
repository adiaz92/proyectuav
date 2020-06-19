#ifndef TRANSFORM_MANAGER_NODE_H
#define TRANSFORM_MANAGER_NODE_H

#include <Eigen/Eigen>
#include <stdio.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <tf2/LinearMath/Quaternion.h>

class TfManagerNode {
 public:
  ros::NodeHandle nh;
  ros::Subscriber fiducial_sub;

  TfManagerNode();
  void parseMarker(int ID);
  void createTransform(geometry_msgs::Transform marker_wrt_camera);
  void fiducial_callBack(fiducial_msgs::FiducialTransformArray fiducial_msg);

  std::vector<double> pose_vector;
  geometry_msgs::Pose marker_pose;

  tf::TransformListener listener;
};


#endif
