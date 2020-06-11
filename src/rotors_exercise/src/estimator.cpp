#include "estimator.h"

EstimatorNode::EstimatorNode() {

  ros::NodeHandle nh;

  ROS_INFO ("Creating data structures for the Kalman Filter"); 

  /* BEGIN: Instantiate here the matrices and vectors of the Kalman filter*/

	// check documentation 

  /* END: Instantiate here the matrices and vectors of the Kalman filter*/

  /* BEGIN: Set the initial conditions*/
  ROS_INFO ("Set the initial conditions "); 

  /* END: Set the initial conditions*/

  ROS_INFO ("Creating subscribers, publisher and timer"); 

  // subscriber
  ground_truth_pose_sub_= nh.subscribe("/firefly/ground_truth/pose_with_covariance", 1, &EstimatorNode::GroundTruthPoseWithCovarianceStampedCallback, this);
  pose_sub_             = nh.subscribe("/firefly/fake_gps/pose_with_covariance", 1, &EstimatorNode::PoseWithCovarianceStampedCallback, this);
  imu_sub_              = nh.subscribe("/firefly/imu", 1, &EstimatorNode::ImuCallback, this);
  
  // publisher
  pose_with_covariance_pub_  = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/firefly/pose_with_covariance", 1);

  // timer
  time_reference = ros::WallTime::now(); 
  timer_         = nh.createTimer(ros::Duration(1), &EstimatorNode::TimedCallback, this);
}

EstimatorNode::~EstimatorNode() { }

void EstimatorNode::predict (Eigen::Vector3d u, double dT)
{
  //publish your data
  //ROS_INFO("Publishing ...");

  /* BEGIN: Compute F, G and Q for dt  and run prediction step computing x-, P-, z- */
 

  /* END: Compute F, G and Q for dt */

  // Store the current best estimate based on prediction 
  //  x_hat = ...; 
  //  P_hat = ...; 


	// Print all the debugging info you may need
  ROS_INFO ("\n\n");
  ROS_INFO ("Debugging prediction step (dt= %0.4fsec)", dT);

}

void EstimatorNode::update  (Eigen::Vector3d z)
{
    /* BEGIN: Compute K, X+ and P+*/

    
    /* END: Compute K, X+ and P+*/

     // Store the current best estimate based on prediction 
	 //  x_hat = ...; 
	 //  P_hat = ...; 

	// Print all the debugging info you may need
	ROS_INFO ("\n\n");
	ROS_INFO ("Debugging update step");
    
}

void EstimatorNode::ImuCallback(
    const sensor_msgs::ImuConstPtr& imu_msg) {

  ROS_INFO_ONCE("Estimator got first IMU message.");

  incomingImuMsg_ = *imu_msg; 

  if (calibrating)
  {
    calib_imu_att_q_buffer.push_back(tf::Quaternion(imu_msg->orientation.x,imu_msg->orientation.y,imu_msg->orientation.z,imu_msg->orientation.w));
    calib_imu_ang_vel_buffer.push_back(tf::Vector3(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z));
    calib_imu_accel_buffer.push_back(tf::Vector3(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z));

    time_reference = ros::WallTime::now(); 
  }else
  {
    msgPoseWithCovariance_.header.stamp = imu_msg->header.stamp;
     
    /* BEGIN: Process the acceleration: remove bias, rotate and remove gravity*/
            
    /* END: Process the acceleration: remove bias, rotate and remove gravity*/

	// Compute delta T
    double dT = (ros::WallTime::now()-time_reference).toSec();
    
	  Eigen::Vector3d u ; //filled it in with accel
	
    predict(u, dT);

    publishPose(); 
    time_reference = ros::WallTime::now();     

  }
}

void EstimatorNode::GroundTruthPoseWithCovarianceStampedCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {

  ROS_INFO_ONCE("Estimator got first ground truth pose message.");

  incomingGroundTruthPoseMsg_ = *pose_msg; 
}

void EstimatorNode::PoseWithCovarianceStampedCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {

  ROS_INFO_ONCE("Estimator got first pose message.");

  incomingPoseMsg_ = *pose_msg; 

  if (calibrating)
  {
    calib_pose_sensor_att_buffer.push_back(tf::Quaternion(pose_msg->pose.pose.orientation.x,
                                                          pose_msg->pose.pose.orientation.y,
                                                          pose_msg->pose.pose.orientation.z,
                                                          pose_msg->pose.pose.orientation.w));
    calib_pose_sensor_pos_buffer.push_back(tf::Vector3( pose_msg->pose.pose.position.x,
                                                        pose_msg->pose.pose.position.y,
                                                        pose_msg->pose.pose.position.z));
  }else
  {
    msgPoseWithCovariance_.header.stamp = pose_msg->header.stamp;

    /* BEGIN: Generate the measurement z and call update*/
    Eigen::Vector3d z ( pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z );
    update(z); 
    /* END: Generate the measurement z and call update*/

    publishPose();  
  }

  
}

void EstimatorNode::publishPose()
{
  //publish your data
  //ROS_INFO("Publishing ...");
  msgPoseWithCovariance_.header.frame_id = "world";

  // publish also as pose with covariance 
  msgPoseWithCovariance_.pose.pose.position.x = 0.0;
  msgPoseWithCovariance_.pose.pose.position.y = 0.0;
  msgPoseWithCovariance_.pose.pose.position.z = 0.0;
  // Take the orientation directly from IMU since we don't estimate it 
  msgPoseWithCovariance_.pose.pose.orientation = incomingImuMsg_.orientation;

  // fill in the values corresponding to position in the covariance
  //msgPoseWithCovariance_.pose.covariance[0] = ...
  
  pose_with_covariance_pub_.publish(msgPoseWithCovariance_);

}

void EstimatorNode::TimedCallback(
      const ros::TimerEvent& e){
   ROS_INFO_ONCE("Timer initiated.");
   //publishPose();
}

void  EstimatorNode::startCalibration () { 
  calibrating = true; 
  ROS_INFO_ONCE("Calibration initiated.");
}
void  EstimatorNode::endCalibration ()   { 

  imu_att_q_bias   = averageQuaternion(calib_imu_att_q_buffer);
  imu_ang_vel_bias = averageVector3(calib_imu_ang_vel_buffer);
  imu_accel_bias   = averageVector3(calib_imu_accel_buffer);
  pose_sensor_pos_offset = averageVector3(calib_pose_sensor_pos_buffer);
  pose_sensor_att_bias   = averageQuaternion(calib_pose_sensor_att_buffer);

  pose2imu_rotation = imu_att_q_bias.inverse() * pose_sensor_att_bias;

  calibrating = false;
  ROS_INFO_ONCE("Calibration ended. Summary: ");
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("IMU samples %d Pose samples %d", (int)calib_imu_att_q_buffer.size(), (int)calib_pose_sensor_pos_buffer.size());
  double roll, pitch, yaw;
  tf::Matrix3x3(imu_att_q_bias).getRPY(roll, pitch, yaw);
  ROS_INFO("IMU RPY bias %f %f %f", roll, pitch, yaw);
  ROS_INFO("IMU Ang.Vel bias %f %f %f", imu_ang_vel_bias.x(), imu_ang_vel_bias.y(), imu_ang_vel_bias.z());
  ROS_INFO("IMU Accel. bias %f %f %f", imu_accel_bias.x(), imu_accel_bias.y(), imu_accel_bias.z());
  ROS_INFO("Pose Sensor pose bias %f %f %f", pose_sensor_pos_offset.x(), pose_sensor_pos_offset.y(), pose_sensor_pos_offset.z());
  tf::Matrix3x3(pose_sensor_att_bias).getRPY(roll, pitch, yaw);
  ROS_INFO("Pose Sensor RPY bias %f %f %f", roll, pitch, yaw);
  tf::Matrix3x3(pose2imu_rotation).getRPY(roll, pitch, yaw);
  ROS_INFO("Offset Pose to IMU RPY %f %f %f", roll, pitch, yaw);
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  
  // free memory
  calib_imu_att_q_buffer.clear(); 
  calib_imu_ang_vel_buffer.clear(); 
  calib_imu_accel_buffer.clear(); 
  calib_pose_sensor_att_buffer.clear(); 
  calib_pose_sensor_pos_buffer.clear(); 
}

tf::Quaternion EstimatorNode::averageQuaternion(std::vector<tf::Quaternion> vec) // It is hacky to do it in RPY
{  
  if (vec.size() == 0) 
    return tf::Quaternion();

  double roll, pitch, yaw;
  double calib_roll, calib_pitch, calib_yaw;
  calib_roll = calib_pitch = calib_yaw = 0.0;
  for(int i = 0; i < vec.size(); i++) 
  {
    tf::Matrix3x3(vec[i]).getRPY(roll, pitch, yaw);
    calib_roll += roll;
    calib_pitch += pitch;
    calib_yaw += yaw;
  }
  calib_roll  = calib_roll / (double)vec.size();
  calib_pitch = calib_pitch / (double)vec.size();
  calib_yaw   = calib_yaw / (double)vec.size();

  return tf::createQuaternionFromRPY(calib_roll, calib_pitch, calib_yaw);
}

tf::Vector3 EstimatorNode::averageVector3(std::vector<tf::Vector3> vec)
{
  if (vec.size() == 0) 
    return tf::Vector3();

  tf::Vector3 res(0.0, 0.0, 0.0);
  for(int i = 0; i < vec.size(); i++) 
  {
    res += vec[i];
  }
  res /= vec.size();

  return res;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "estimator");

  ROS_INFO ("Starting estimator node"); 

  EstimatorNode estimator_node;

  ROS_INFO("Waiting for simulation to start up...");  
  while (ros::WallTime::now().toSec() < 0.2)
  {
    ROS_INFO("Waiting for simulation to start up...");  
    ros::spinOnce();
    ros::Duration(0.01).sleep();     
  }

  //give time to start up the simulation
  ros::Duration(0.2).sleep();     

  // Initialize your filter / controller.
  ROS_INFO("Calibrating offsets for 2 secs...");
  estimator_node.startCalibration(); 
  // 2 secs for init the filters
  ros::WallTime time_reference = ros::WallTime::now();
  while ((ros::WallTime::now()-time_reference).toSec() < 2.0)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();     
  }

  estimator_node.endCalibration(); 

  // let it go .. 
  ros::spin();

  return 0;
}
