#include "estimator.h"

EstimatorNode::EstimatorNode() {

  ros::NodeHandle nh;

  /* Read parameters from the parameter server */
  ROS_INFO ("Reading Parameters from the parameter server");

  readPars(nh);

  ROS_INFO ("Creating data structures for the Kalman Filter");

   /* BEGIN: Instantiate here the matrices and vectors of the Kalman filter*/

	// check documentation
  A.resize(7,7);
  B.resize(7,4);
  Q.resize(7,7);
  L.resize(7,7);
  u.resize(4);
  yp.resize(4);
  yv.resize(3);

  /* END: Instantiate here the matrices and vectors of the Kalman filter*/

  /* BEGIN: Set the constants C R*/
  ROS_INFO ("Set constants");

  Cp.resize(4,7);
  Cp << 1, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0;

  Cv.resize(3,7);
  Cv << 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 1;

  gravity_w = tf::Vector3(0.,0.,-9.8);

  Rp.resize(4,4);
  Rp << 0.01, 0, 0, 0,
       0, 0.01, 0, 0,
       0, 0, 0.01, 0,
       0, 0, 0, 0.05;

  Rv.resize(3,3);

  Lp.resize(7,4);
  Lv.resize(7,3);
  /* END: Set the constants*/

  x_hat.resize(7);

  x_hat << -2.5, 0, 0, 0, 0, 0, 0;
  u << 0, 0, 0, 0;

    /* BEGIN: Set the initial conditions: P_hat0*/
  ROS_INFO ("Set the initial conditions ");
  P_hat.resize(7,7);

  P_hat << 1, 0, 0, 0, 0 ,0, 0,
           0, 1, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0,
           0, 0, 0, 1, 0, 0, 0,
           0, 0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 0, 1;

  //P_hat = P_hat * sigma_sqr_P0;
  /* END: Set the initial conditions*/

  ROS_INFO ("Creating subscribers, publisher and timer");

  // subscriber
  pose_sub_             = nh.subscribe("/pose_estimation_aruco", 1, &EstimatorNode::MarkerPoseCallback, this);
  odometry_sub_         = nh.subscribe("/firefly/fake_twist", 1, &EstimatorNode::OdometryCallback, this);
  imu_sub_              = nh.subscribe("/firefly/imu", 1, &EstimatorNode::ImuCallback, this);

  // publisher
  odometry_pub_  = nh.advertise<nav_msgs::Odometry>("/firefly/odom_filtered", 1);

  // timer
  time_reference = ros::WallTime::now();

}

EstimatorNode::~EstimatorNode() { }

void EstimatorNode::predict ()
{
  //publish your data
  //ROS_INFO("Publishing ...");

  ros::WallTime time_reference_new = ros::WallTime::now();
  double dT = (time_reference_new-time_reference).toSec();

  /* BEGIN: Compute A, B and Q for dt  and run prediction step computing x-, P-, z- */
  A << 1, 0, 0, 0, dT, 0 ,0,
       0, 1, 0, 0, 0, dT, 0,
       0, 0, 1, 0, 0, 0, dT,
       0, 0, 0, 1, 0, 0, 0,
       0, 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 0, 1;

  B << std::pow(dT,2) / 2, 0, 0, 0,
       0, std::pow(dT,2) / 2, 0, 0,
       0, 0, std::pow(dT,2) / 2, 0,
                    0, 0, 0, dT,
                   dT, 0, 0, 0,
                   0, dT, 0, 0,
                   0, 0, dT, 0;

  double val_sigma_dT = 0.5 * std::pow(dT,2) / 2;
  double val_sigma_dT_2 = 0.3 * std::pow(dT,2) / 2;

  Q << val_sigma_dT * dT, 0, 0, 0, val_sigma_dT, 0, 0,
       0, val_sigma_dT * dT, 0, 0, 0, val_sigma_dT, 0,
       0, 0, val_sigma_dT * dT, 0, 0, 0, val_sigma_dT,
       0, 0, 0, val_sigma_dT_2, 0, 0, 0,
       val_sigma_dT, 0, 0, 0, val_sigma_dT, 0, 0,
       0, val_sigma_dT, 0, 0, 0, val_sigma_dT, 0,
       0, 0, val_sigma_dT, 0, 0, 0, val_sigma_dT;

  x_hat = A*x_hat + B*u;
  //ROS_INFO("x_hat predict 0: %f", x_hat[0]);
  //ROS_INFO("x_hat predict 1: %f", x_hat[1]);
  //ROS_INFO("x_hat predict 2: %f", x_hat[2]);
  //ROS_INFO("x_hat predict 3: %f", x_hat[3]);
  //ROS_INFO("x_hat predict 4: %f", x_hat[4]);
  //ROS_INFO("x_hat predict 5: %f", x_hat[5]);
  //ROS_INFO("x_hat predict 6: %f", x_hat[6]);
  P_hat = A*P_hat*A.transpose() + Q;
  /* END: Compute A, B and Q for dt and end of running prediction step computing x-, P-, z-*/

  time_reference = time_reference_new;

	// Print all the debugging info you may need

//  ROS_INFO ("\n\n");
//  ROS_INFO ("Debugging prediction step (dt= %0.4fsec)", dT);
//  ROS_INFO("Pos estimation %f %f %f", x_hat(0),x_hat(1),x_hat(2));
//  ROS_INFO("Vel estimation %f %f %f", x_hat(3),x_hat(4),x_hat(5));
}

void EstimatorNode::update  (int sensor)
{

    /* Sensors:
      - Position based on visual information
      - Velocity (using fake odometry)
    */

    switch(sensor)
    {
      case 1:
        Lp = (Cp*P_hat).transpose() * (Cp*P_hat*Cp.transpose() + Rp).inverse();
        x_hat = x_hat + Lp*(yp-Cp*x_hat);
        P_hat = P_hat - Lp*Cp*P_hat;
        //ROS_INFO("yp 0: %f", yp[0]);
        //ROS_INFO("yp 1: %f", yp[1]);
        //ROS_INFO("yp 2: %f", yp[2]);
        //ROS_INFO("yp 3: %f", yp[3]);
        //ROS_INFO("x_hat pose 0: %f", x_hat[0]);
        //ROS_INFO("x_hat pose 1: %f", x_hat[1]);
        //ROS_INFO("x_hat pose 2: %f", x_hat[2]);
        //ROS_INFO("x_hat pose 3: %f", x_hat[3]);
        //ROS_INFO("x_hat pose 4: %f", x_hat[4]);
        //ROS_INFO("x_hat pose 5: %f", x_hat[5]);
        //ROS_INFO("x_hat pose 6: %f", x_hat[6]);
        break;
      case 2:
        Lv = (Cv*P_hat).transpose() * (Cv*P_hat*Cv.transpose() + Rv).inverse();
        x_hat = x_hat + Lv*(yv-Cv*x_hat);
        P_hat = P_hat - Lv*Cv*P_hat;
        break;
    }

	// Print all the debugging info you may need
  /*
	ROS_INFO ("\n\n");
	ROS_INFO ("Debugging update step");
  ROS_INFO("Pos estimation %f %f %f", x_hat(0),x_hat(1),x_hat(2));

  ROS_INFO("Vel estimation %f %f %f", x_hat(3),x_hat(4),x_hat(5));
  */

}

void EstimatorNode::ImuCallback(
    const sensor_msgs::ImuConstPtr& imu_msg) {

  //ROS_INFO_ONCE("Estimator got first IMU message.");

  incomingImuMsg_ = *imu_msg;

  if (calibrating)
  {
    calib_imu_att_q_buffer.push_back(tf::Quaternion(imu_msg->orientation.x,imu_msg->orientation.y,imu_msg->orientation.z,imu_msg->orientation.w));
    calib_imu_ang_vel_buffer.push_back(tf::Vector3(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z));
    calib_imu_accel_buffer.push_back(tf::Vector3(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z));

    time_reference = ros::WallTime::now();
  }else
  {
    msgOdometry_.header.stamp = imu_msg->header.stamp;
    //ROS_INFO("imu 0: %f", imu_msg->linear_acceleration.x);
    //ROS_INFO("imu 1: %f", imu_msg->linear_acceleration.y);
    //ROS_INFO("imu 2: %f", imu_msg->linear_acceleration.z);
    /* BEGIN: Process the acceleration: remove bias, rotate and remove gravity*/
    tf::Vector3 imu_ang_vel ( imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    tf::Vector3     imu_accel ( imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    tf::Quaternion  imu_att_q(imu_msg->orientation.x,imu_msg->orientation.y,imu_msg->orientation.z,imu_msg->orientation.w);

    imu_att_q = pose2imu_rotation*imu_att_q;
    //+imu_accel_bias;
    //ROS_INFO("imuatq 0: %f", imu_att_q[0]);
    //ROS_INFO("imuatq 1: %f", imu_att_q[1]);
    //ROS_INFO("imuatq 2: %f", imu_att_q[2]);
    //ROS_INFO("imuatq 3: %f", imu_att_q[3]);

    //ROS_INFO("gravity_w 0: %f", gravity_w[0]);
    //ROS_INFO("gravity_w 1: %f", gravity_w[1]);
    //ROS_INFO("gravity_w 2: %f", gravity_w[2]);
    tf::Vector3 Prueba;
    Prueba = tf::quatRotate(imu_att_q, imu_accel);
    //ROS_INFO("Prueba0: %f", Prueba[0]);
    //ROS_INFO("Prueba1: %f", Prueba[1]);
    //ROS_INFO("Prueba2: %f", Prueba[2]);
    imu_accel = gravity_w + tf::quatRotate(imu_att_q, imu_accel);
    imu_ang_vel = tf::quatRotate(imu_att_q, imu_ang_vel+imu_ang_vel_bias);


    /* END: Process the acceleration: remove bias, rotate and remove gravity*/
	  //u << imu_accel[0], imu_accel[1], imu_accel[2], imu_ang_vel[2];
    u << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z - 9.8, imu_ang_vel[2];
    //ROS_INFO("u 0: %f", u[0]);
    //ROS_INFO("u 1: %f", u[1]);
    //ROS_INFO("u 2: %f", u[2]);
    //ROS_INFO("u 3: %f", u[3]);
    //ROS_INFO("ut 0: %f", imu_accel[0]);
    //ROS_INFO("ut 1: %f", imu_accel[1]);
    //ROS_INFO("ut 2: %f", imu_accel[2]);
    //ROS_INFO("imu_accel_bias 0: %f", imu_accel_bias[0]);
    //ROS_INFO("imu_accel_bias 1: %f", imu_accel_bias[1]);
    //ROS_INFO("imu_accel_bias 2: %f", imu_accel_bias[2]);



    //if(calibrated) ros::Duration(20.0).sleep();

    predict();

    publishPose();

  }
}

void EstimatorNode::MarkerPoseCallback(
  const geometry_msgs::PoseStamped& pose_msg) {

  //ROS_INFO("ARUCO INFO RECEIVED");
  incomingPoseMsg_ = pose_msg.pose;

  msgOdometry_.header.stamp = pose_msg.header.stamp;

  /* BEGIN: Generate the measurement y and call update*/
  yp << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, pose_msg.pose.orientation.z;

  predict();

  update(1);

  /* END: Generate the measurement y and call update*/

  publishPose();

  //ROS_INFO("ARUCO DONE");

}

void EstimatorNode::OdometryCallback(
    const geometry_msgs::TwistWithCovarianceStamped& odometry_msg) {

  //ROS_INFO("ODOMETRY RECEIVED");
  //incomingOdomMsg_ = *odometry_msg;

  msgOdometry_.header.stamp = ros::Time::now();

  yv << odometry_msg.twist.twist.linear.x, odometry_msg.twist.twist.linear.y, odometry_msg.twist.twist.linear.z;
  //ROS_INFO("Yv 0: %f", yv[0]);
  //ROS_INFO("Yv 1: %f", yv[1]);
  //ROS_INFO("Yv 2: %f", yv[2]);

  //As we only care about the linear twist we will not take the angular covariance (at this moment, will see later for yawl)
  Rv << odometry_msg.twist.covariance[0], odometry_msg.twist.covariance[1], odometry_msg.twist.covariance[2],
        odometry_msg.twist.covariance[6], odometry_msg.twist.covariance[7], odometry_msg.twist.covariance[8],
        odometry_msg.twist.covariance[12], odometry_msg.twist.covariance[13], odometry_msg.twist.covariance[14];

  predict();

  update(2);

  publishPose();


}



void EstimatorNode::publishPose()
{
  //publish your data
  //ROS_INFO("Publishing ...");
  msgOdometry_.header.frame_id = "world";
  msgOdometry_.child_frame_id = "imu";

  // publish also as pose with covariance
  msgOdometry_.pose.pose.position.x = x_hat[0];
  msgOdometry_.pose.pose.position.y = x_hat[1];
  msgOdometry_.pose.pose.position.z = x_hat[2];

  msgOdometry_.twist.twist.linear.x = x_hat[3];
  msgOdometry_.twist.twist.linear.y = x_hat[4];
  msgOdometry_.twist.twist.linear.z = x_hat[5];

  // Take the orientation directly from IMU since we don't estimate it
  msgOdometry_.pose.pose.orientation = incomingImuMsg_.orientation;

  // fill in the values corresponding to position in the covariance

  for (int ii = 0; ii <3; ii++){

    for (int jj = 0; jj<3; jj++){
      msgOdometry_.pose.covariance[ii*6+jj] = P_hat(ii,jj);
      msgOdometry_.twist.covariance[ii*6+jj] = P_hat(ii+3,jj+3);
    }

  }

  odometry_pub_.publish(msgOdometry_);

}

void  EstimatorNode::readPars (ros::NodeHandle& nh) {
    //nh.getParam("estimation/gps_covariance", sigma_sqr_gps);
    //nh.getParam("estimation/process_covariance", sigma_sqr_process);
    //nh.getParam("estimation/initial_error_covariance", sigma_sqr_P0);
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

  pose2imu_rotation = pose_sensor_att_bias*imu_att_q_bias.inverse();
  imu_accel_bias = -imu_accel_bias - tf::quatRotate( (pose2imu_rotation*imu_att_q_bias).inverse(), gravity_w);

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

  // Set initial state values
  x_hat << pose_sensor_pos_offset[0], pose_sensor_pos_offset[1], pose_sensor_pos_offset[2], 0.0, 0.0, 0.0, 0.0;

  // free memory
  calib_imu_att_q_buffer.clear();
  calib_imu_ang_vel_buffer.clear();
  calib_imu_accel_buffer.clear();
  calib_pose_sensor_att_buffer.clear();
  calib_pose_sensor_pos_buffer.clear();

  calibrated = true;
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

  ROS_INFO("Calibration DONE");
  // let it go ..
  ros::spin();

  return 0;
}
