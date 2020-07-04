#include "transform_pose.h"

TfPose::TfPose()
{
  estimated_position_sub = nh.subscribe("/firefly/odom_filtered", 1, &TfPose::estimated_position_callBack, this);
  mission_sub = nh.subscribe("/firefly/transform_pose/mission", 1, &TfPose::mission_callBack, this);
  ventral_fiducial_sub = nh.subscribe("/aruco_detect_ventral/fiducial_transforms", 1, &TfPose::ventral_fiducial_callBack, this);
  frontal_fiducial_sub = nh.subscribe("/aruco_detect_frontal/fiducial_transforms", 1, &TfPose::frontal_fiducial_callBack, this);
  pose_estimation_pub = nh.advertise<geometry_msgs::Point>("/firefly/transform_pose/estimated_point", 1000, true);
}

void TfPose::frontal_fiducial_callBack(fiducial_msgs::FiducialTransformArray fiducial_msg)
{
  if (frontal_search)
  {
    ROS_INFO("FRONTAL SEARCH");
    ROS_INFO("TARGET ID = %d", target_id);
    if (fiducial_msg.transforms.size() > 0 )
    {
      //extract the ID from the fiducial message
      int id = fiducial_msg.transforms[0].fiducial_id;

      if (id == target_id)
      {
        ROS_INFO("DETECTED");
        geometry_msgs::Transform marker_wrt_camera_geometry = fiducial_msg.transforms[0].transform;
        tf::Transform  marker_wrt_camera;
        tf::transformMsgToTF(marker_wrt_camera_geometry, marker_wrt_camera);
        //create the transform
        tf::Transform base_link_wrt_world;
        tf::Vector3 base_link_position_vector(base_link_pose.position.x, base_link_pose.position.y,
                                              base_link_pose.position.z);
        tf::Quaternion base_link_orientation_quaternion(base_link_pose.orientation.x, base_link_pose.orientation.y,
                                                        base_link_pose.orientation.z, base_link_pose.orientation.w);
        base_link_wrt_world.setOrigin(base_link_position_vector);
        base_link_wrt_world.setRotation(base_link_orientation_quaternion);
        createTransform(base_link_wrt_world, marker_wrt_camera);
      }
      frontal_search = false;
    }
  }
}

void TfPose::ventral_fiducial_callBack(fiducial_msgs::FiducialTransformArray fiducial_msg)
{
  if (ventral_search)
  {
    if (fiducial_msg.transforms.size() > 0 )
    {
      //extract the ID from the fiducial message
      int id = fiducial_msg.transforms[0].fiducial_id;

      if (id == target_id)
      {
        geometry_msgs::Transform marker_wrt_camera_geometry = fiducial_msg.transforms[0].transform;
        tf::Transform  marker_wrt_camera;
        tf::transformMsgToTF(marker_wrt_camera_geometry, marker_wrt_camera);
        //create the transform
        tf::Transform base_link_wrt_world;
        tf::Vector3 base_link_position_vector(base_link_pose.position.x, base_link_pose.position.y,
                                              base_link_pose.position.z);
        tf::Quaternion base_link_orientation_quaternion(base_link_pose.orientation.x, base_link_pose.orientation.y,
                                                        base_link_pose.orientation.z, base_link_pose.orientation.w);
        base_link_wrt_world.setOrigin(base_link_position_vector);
        base_link_wrt_world.setRotation(base_link_orientation_quaternion);
        createTransform(base_link_wrt_world, marker_wrt_camera);
      }
    }
    ventral_search = false;
  }
}


void TfPose::mission_callBack(mission_planner::transform_type transform_type)
{
  frontal_search = transform_type.frontal_search;
  ventral_search = transform_type.ventral_search;
  target_id = transform_type.target_id;
}

void TfPose::estimated_position_callBack(nav_msgs::Odometry estimated_odom)
{
  base_link_pose = estimated_odom.pose.pose;
}

void TfPose::createTransform(tf::Transform base_link_wrt_world, tf::Transform marker_wrt_camera)
{
  tf::StampedTransform camera_wrt_base_link_stamped;
  try{
    listener.lookupTransform("/firefly/base_link", "/firefly/camera_ventral_optical_link", ros::Time(0), camera_wrt_base_link_stamped);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }
  //Get the position of the camera wrt base_link
  tf::Transform camera_wrt_base_link;
  camera_wrt_base_link.setOrigin(camera_wrt_base_link_stamped.getOrigin());
  camera_wrt_base_link.setRotation(camera_wrt_base_link_stamped.getRotation());

  //Get the position of the camera wrt to world
  tf::Transform camera_wrt_world;
  camera_wrt_world.mult(base_link_wrt_world, camera_wrt_base_link);

  //Get the position of the marker wrt to world
  tf::Transform marker_wrt_world;
  marker_wrt_world.mult(camera_wrt_world, marker_wrt_camera);
  geometry_msgs::Transform marker_wrt_world_geometry;
  tf::transformTFToMsg(marker_wrt_world, marker_wrt_world_geometry);

  //Publish the pose estimation
  geometry_msgs::PoseStamped pose_estimation;
  pose_estimation.pose.position.x = marker_wrt_world_geometry.translation.x;
  pose_estimation.pose.position.y = marker_wrt_world_geometry.translation.y;
  pose_estimation.pose.position.z = marker_wrt_world_geometry.translation.z;
  pose_estimation.pose.orientation = marker_wrt_world_geometry.rotation;
  pose_estimation.header.stamp = ros::Time::now();

  pose_estimation_pub.publish(pose_estimation.pose.position);
}
