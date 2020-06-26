#include "transform_manager.h"

TfManagerNode::TfManagerNode()
{
  nh.getParam("groundMarkers/poseList", pose_vector);
  ROS_INFO("x 1: %f", pose_vector[1]);
  fiducial_sub = nh.subscribe("fiducial_transforms", 1, &TfManagerNode::fiducial_callBack, this);
  pose_estimation_pub = nh.advertise<geometry_msgs::Pose>("pose_estimation_aruco", 1000);
}

tf::Transform TfManagerNode::parseMarker(int ID)
{
  tf::Vector3 marker_translation_vector(pose_vector[ID*6], pose_vector[ID*6+1],pose_vector[ID*6+2]);
  tf::Quaternion marker_orientation_vector;
  marker_orientation_vector.setRPY( pose_vector[ID*6+3], pose_vector[ID*6+4], pose_vector[ID*6+5] );
  tf::Transform marker_wrt_world;
  marker_wrt_world.setOrigin(marker_translation_vector);
  marker_wrt_world.setRotation(marker_orientation_vector);
  return marker_wrt_world;
}

void TfManagerNode::fiducial_callBack(fiducial_msgs::FiducialTransformArray fiducial_msg)
{
  if (fiducial_msg.transforms.size() > 0 )
  {
    //extract the ID from the fiducial message
    int id = fiducial_msg.transforms[0].fiducial_id;
    //call parseMarker to extract the pose
    tf::Transform marker_wrt_world = parseMarker(id);
    geometry_msgs::Transform marker_wrt_camera_geometry = fiducial_msg.transforms[0].transform;
    tf::Transform  marker_wrt_camera;
    tf::transformMsgToTF(marker_wrt_camera_geometry, marker_wrt_camera);
    //create the transform
    createTransform(marker_wrt_world, marker_wrt_camera);
  }
}

void TfManagerNode::createTransform(tf::Transform marker_wrt_world, tf::Transform marker_wrt_camera)
{
  tf::StampedTransform base_link_wrt_camera_stamped;
  try{
    listener.lookupTransform("/firefly/camera_ventral_optical_link", "/firefly/base_link", ros::Time(0), base_link_wrt_camera_stamped);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }
  //Get the position of the base_link wrt to the camera
  tf::Transform base_link_wrt_camera;
  base_link_wrt_camera.setOrigin(base_link_wrt_camera_stamped.getOrigin());
  base_link_wrt_camera.setRotation(base_link_wrt_camera_stamped.getRotation());
  tf::Transform camera_wrt_marker = marker_wrt_camera.inverse();
  //Get the position of the camera wrt to world
  tf::Transform camera_wrt_world;
  camera_wrt_world.mult(marker_wrt_world, camera_wrt_marker);
  geometry_msgs::Transform camera_wrt_world_geometry;
  //Get the position of the base_link wrt to world
  tf::Transform base_link_wrt_world;
  base_link_wrt_world.mult(camera_wrt_world, base_link_wrt_camera);
  geometry_msgs::Transform base_link_wrt_world_geometry;
  tf::transformTFToMsg(base_link_wrt_world, base_link_wrt_world_geometry);
  //Publish the pose estimation
  geometry_msgs::Pose pose_estimation;
  pose_estimation.position.x = base_link_wrt_world_geometry.translation.x;
  pose_estimation.position.y = base_link_wrt_world_geometry.translation.y;
  pose_estimation.position.z = base_link_wrt_world_geometry.translation.z;
  pose_estimation.orientation = base_link_wrt_world_geometry.rotation;
  pose_estimation_pub.publish(pose_estimation);
}
