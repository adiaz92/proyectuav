#ifndef TRANSFORM_POSE_NODE_H
#define TRANSFORM_POSE_NODE_H

#include <Eigen/Eigen>
#include <stdio.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mission_planner/transform_type.h>
#include <nav_msgs/Odometry.h>

class TfPose {
  public:
    ros::NodeHandle nh;
    ros::Subscriber mission_sub;
    ros::Subscriber frontal_fiducial_sub;
    ros::Subscriber ventral_fiducial_sub;
    ros::Subscriber estimated_position_sub;
    ros::Publisher pose_estimation_pub;

    TfPose();

  private:
    bool frontal_search = false;
    bool ventral_search = false;
    int target_id = -1;

    void createTransform(tf::Transform base_link_wrt_world, tf::Transform marker_wrt_camera);
    void estimated_position_callBack(nav_msgs::Odometry estimated_odom);
    void frontal_fiducial_callBack(fiducial_msgs::FiducialTransformArray fiducial_msg);
    void ventral_fiducial_callBack(fiducial_msgs::FiducialTransformArray fiducial_msg);
    void mission_callBack(mission_planner::transform_type transform_type);

    geometry_msgs::Pose base_link_pose;
    mission_planner::transform_type transform_type;

    tf::TransformListener listener;
};


#endif
