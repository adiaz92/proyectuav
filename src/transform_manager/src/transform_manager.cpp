#include "transform_manager.h"

TfManagerNode::TfManagerNode()
{
  nh.getParam("groundMarkers/poseList", pose_vector);
  ROS_INFO("x 1: %f", pose_vector[1]);
  fiducial_sub = nh.subscribe("fiducial_transforms", 1, &TfManagerNode::fiducial_callBack, this);
}

void TfManagerNode::parseMarker(int ID)
{
  marker_pose.position.x = pose_vector[ID*6];
  marker_pose.position.y = pose_vector[ID*6+1];
  marker_pose.position.z = pose_vector[ID*6+2];
  //from rpy to quaternion
  tf2::Quaternion Quaternion_tf2;
  Quaternion_tf2.setRPY( pose_vector[ID*6+3], pose_vector[ID*6+4], pose_vector[ID*6+5] );
  marker_pose.orientation = tf2::toMsg(Quaternion_tf2);
}

void TfManagerNode::fiducial_callBack(fiducial_msgs::FiducialTransformArray fiducial_msg)
{
  if (fiducial_msg.transforms.size() > 0 )
  {
    //extract the ID from the fiducial message
    int id = fiducial_msg.transforms[0].fiducial_id;
    //call parseMarker to extract the pose
    parseMarker(id);
    geometry_msgs::Transform marker_wrt_camera_tf;
    marker_wrt_camera_tf = fiducial_msg.transforms[0].transform;
    //create the transform
    createTransform(marker_wrt_camera_tf);
  }
}

void TfManagerNode::createTransform(geometry_msgs::Transform marker_wrt_camera)
{
  tf::StampedTransform transform_stamped_tf;
  try{
    listener.lookupTransform("/base_link","/firefly/camera_ventral_optical_link", ros::Time(0), transform_stamped_tf);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }

  geometry_msgs::TransformStamped stampedTransform;
  tf::transformStampedTFToMsg(transform_stamped_tf, stampedTransform);
  geometry_msgs::Transform camera_wrt_baselink = stampedTransform.transform;

  //geometry_msgs::Transform

  //Use TF instead of geometry_msgs! 
}
