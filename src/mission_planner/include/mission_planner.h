#ifndef mission_planner_H
#define mission_planner_H

#include <Eigen/Eigen>
#include <stdio.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <final_aerial_project/ProductInfo.h>
#include <final_aerial_project/ProductFeedback.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/SetBool.h>
#include <mission_planner/transform_type.h>

class Mission
{
  public:

    ros::NodeHandle nh;
    ros::NodeHandle n;
    ros::Subscriber request_sub;
    ros::Subscriber estimated_pose_sub;
    ros::Subscriber search_sub;
    ros::Publisher desired_pose_pub;
    ros::Publisher pose_pub;
    ros::Publisher feedback_pub;
    ros::Publisher transform_pub;
    ros::ServiceClient aruco_ventral_client;
    ros::ServiceClient aruco_frontal_client;

    Mission();

    void do_mission();

  private:

    int mission_code;
    int target_id;
    int axis_x_iteration = 0;
    int axis_y_iteration = 0;
    int axis_z_iteration = 0;
    double x_added = 0.5;
    double y_added = -0.5;

    bool waiting_vertical_detection = false;
    bool target_reached_base_pose = false;
    bool target_reached_actual_pose = false;
    bool search_done = false;
    bool going_to_request_pose = false;
    bool going_to_base_pose = false;

    geometry_msgs::PoseWithCovariance approximate_pose;
    geometry_msgs::PoseStamped actual_pose;
    geometry_msgs::PoseStamped base_pose;
    geometry_msgs::Point marker_position;
    std_srvs::SetBool ventral_request;
    std_srvs::SetBool frontal_request;

    final_aerial_project::ProductFeedback feedback;
    mission_planner::transform_type mission_type;

    void activate_camera(int camera_id, bool activate);

    void request_callBack(final_aerial_project::ProductInfo product_info);

    void search_callBack(geometry_msgs::Point search_point);

    void pose_estimation_callback(geometry_msgs::Pose estimated_pose);

    bool equal_pose_with_tolerance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

};

#endif
