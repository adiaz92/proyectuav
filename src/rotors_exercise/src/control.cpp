#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_planning_msgs/PolynomialTrajectory.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

#include <tf/tf.h>

#include <dynamic_reconfigure/server.h>
#include <rotors_exercise/ControllerConfig.h>

#define M_PI  3.14159265358979323846  /* pi */

//Period for the control loop
float control_loop_period = 0.01;

//Elapsed time between pose messages
float 		delta_time_pose 	  = 0.0;
ros::Time	latest_pose_update_time;


// Feedbacks
sensor_msgs::Imu 							latest_imu;
geometry_msgs::PoseWithCovarianceStamped	latest_pose;

nav_msgs::Path								latest_trajectory;
int 										current_index;

// Setpoints
tf::Vector3					setpoint_pos;
tf::Vector3					prev_setpoint_pos;
double						setpoint_yaw;

tf::Vector3					error_pos;
double						error_yaw;

tf::Vector3					command_pos;
double						command_yaw;

tf::Vector3					integral_error;
double						integral_error_yaw;

tf::Vector3					d_error;
double						d_error_yaw;

tf::Vector3					previous_error_pos;
double						previous_error_yaw;

// Gravity
double 	gravity_compensation = 0.0 ;
float 	gravity              = 9.54;

// PID control gains and limits
double  x_kp, x_ki, x_integral_limit, x_kd, x_integral_window_size,
			y_kp, y_ki, y_kd, y_integral_limit, y_integral_window_size,
			z_kp, z_ki, z_kd, z_integral_limit, z_integral_window_size,
			yaw_kp, yaw_ki, yaw_kd, yaw_integral_limit, yaw_integral_window_size,
			x_vel_limit, y_vel_limit, z_vel_limit, yaw_vel_limit;


// Velocity commands and limits
float x_raw_vel_cmd, y_raw_vel_cmd, z_raw_vel_cmd, yaw_raw_vel_cmd;
float maxXVel, maxYVel, maxZVel, maxYawVel;
float x_vel_cmd, y_vel_cmd, z_vel_cmd, yaw_vel_cmd;

// Acceleration feedback for feedforward
tf::Vector3		body_accel;

// publisher to confirm current trajectory
ros::Publisher trajectory_pub;

void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
	ROS_INFO_ONCE("First Imu msg received ");
	latest_imu = *msg; // Handle IMU data.
}
void poseCallback(nav_msgs::Odometry msg)
{
	ROS_INFO_ONCE("First Pose msg received ");

	latest_pose.pose = msg.pose;
	// Handle pose measurements.

}

/* This function receives a trajectory of type MultiDOFJointTrajectoryConstPtr from the waypoint_publisher
	and converts it to a Path in "latest_trajectory" to send it to rviz and use it to fly*/
void MultiDOFJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {

    // Clear all pending waypoints.
  latest_trajectory.poses.clear();

  // fill the header of the latest trajectory
  latest_trajectory.header = msg->header;
  latest_trajectory.header.frame_id = "world" ;

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  ROS_INFO("New trajectory with %d waypoints", (int) n_commands);

  // Extract the waypoints and print them
  for (size_t i = 0; i < n_commands; ++i) {

    geometry_msgs::PoseStamped wp;
    wp.pose.position.x    = msg->points[i].transforms[0].translation.x;
    wp.pose.position.y    = msg->points[i].transforms[0].translation.y;
    wp.pose.position.z    = msg->points[i].transforms[0].translation.z;
    wp.pose.orientation = msg->points[i].transforms[0].rotation;

		geometry_msgs::PoseStamped prev_wp;
		if (i=0){

		}
    latest_trajectory.poses.push_back(wp);

    ROS_INFO ("WP %d\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t", (int)i,
    	wp.pose.position.x, wp.pose.position.y, wp.pose.position.z, tf::getYaw(wp.pose.orientation));
  }
  current_index = 0;
}

/* This function receives a waypointList of type WaypointList from the voxblox_rrt_planner
	and converts it to a Path in "latest_trajectory" */

void WaypointListCallback(const geometry_msgs::PoseArray& msg){
	// Clear all pending waypoints.
		latest_trajectory.poses.clear();

	// fill the header of the latest trajectory
		latest_trajectory.header = msg.header;
		latest_trajectory.header.frame_id = "world" ;

		const size_t n_commands = msg.poses.size();

		if(n_commands < 1){
			ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
			return;
	  	}

		ROS_INFO(" Got a list of waypoints, %zu long!",(int) n_commands);

	  // Extract the waypoints and print them
    for (size_t i = 0; i < n_commands; ++i) {
			ROS_INFO("%zu waypoint!",i);
        geometry_msgs::PoseStamped wp;
        wp.pose.position.x    = msg.poses[i].position.x;
        wp.pose.position.y    = msg.poses[i].position.y;
        wp.pose.position.z    = msg.poses[i].position.z;
        wp.pose.orientation = msg.poses[i].orientation;

        latest_trajectory.poses.push_back(wp);

    ROS_INFO ("WP %d\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t", (int)i,
    	wp.pose.position.x, wp.pose.position.y, wp.pose.position.z, atan(wp.pose.position.y/wp.pose.position.x));
  }
	current_index = 0;
}


/// Dynamic reconfigureCallback
void reconfigure_callback(rotors_exercise::ControllerConfig &config, uint32_t level)
{
	// Copy new configuration
	// m_config = config;

	gravity_compensation = config.gravity_compensation;

	x_kp = config.x_kp;
	x_ki = config.x_ki;
	x_kd = config.x_kd;
	x_integral_limit = config.x_integral_limit;
	//x_integral_window_size = config.x_integral_window_size;

	y_kp = config.y_kp;
	y_ki = config.y_ki;
	y_kd = config.y_kd;
	y_integral_limit = config.y_integral_limit;
  //y_integral_window_size = config.y_integral_window_size;

	z_kp = config.z_kp;
	z_ki = config.z_ki;
	z_kd = config.z_kd;
	z_integral_limit = config.z_integral_limit;
	//z_integral_window_size = config.z_integral_window_size;

	yaw_kp = config.yaw_kp;
	yaw_ki = config.yaw_ki;
	yaw_kd = config.yaw_kd;
	yaw_integral_limit = config.yaw_integral_limit;
  //yaw_integral_window_size = config.yaw_integral_window_size;

	x_vel_limit = config.x_vel_limit;
	y_vel_limit = config.y_vel_limit;
	z_vel_limit = config.z_vel_limit;
	yaw_vel_limit = config.yaw_vel_limit;

	maxXVel		= config.x_vel_limit;
	maxYVel		= config.y_vel_limit;
	maxZVel		= config.z_vel_limit;
	maxYawVel	= config.yaw_vel_limit;

	ROS_INFO (" ");
	ROS_INFO ("Reconfigure callback have been called with new Settings ");

}

void timerCallback(const ros::TimerEvent& e)
{
	double roll, pitch, yaw;
	if (latest_pose.header.stamp.nsec > 0.0)
	{
		ROS_INFO ("///////////////////////////////////////");

		// ADD here any debugging you need
		ROS_INFO ("WP \t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t",
			setpoint_pos[0],
			setpoint_pos[1],
			setpoint_pos[2],
			setpoint_yaw );

		ROS_INFO ("Pose\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t",
			latest_pose.pose.pose.position.x,
			latest_pose.pose.pose.position.y,
			latest_pose.pose.pose.position.z,
			tf::getYaw(latest_pose.pose.pose.orientation));
		ROS_INFO ("PosErr\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t",
			error_pos[0],
			error_pos[1],
			error_pos[2],
			error_yaw);
		ROS_INFO ("PrPosErr\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t",
			previous_error_pos[0],
			previous_error_pos[1],
			previous_error_pos[2],
			previous_error_yaw);
		ROS_INFO ("IntgrE\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t",
			integral_error[0],
			integral_error[1],
			integral_error[2],
			integral_error_yaw );
		ROS_INFO ("DerivE\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t",
			d_error[0],
			d_error[1],
			d_error[2],
			d_error_yaw );
		ROS_INFO ("Action\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t",
			x_vel_cmd,
			y_vel_cmd,
			z_vel_cmd,
			yaw_vel_cmd);
		ROS_INFO ("..................................... ");



		trajectory_pub.publish(latest_trajectory);
	}
}

tf::Vector3 rotateZ (tf::Vector3 input_vector, float angle)
{
	tf::Quaternion quat;
	quat.setRPY(0.0, 0.0, angle);
	tf::Transform transform (quat);

	return (transform * input_vector);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	ros::NodeHandle nh_params("~");

	ROS_INFO("running controller");

	// Inputs: imu and pose messages, and the desired trajectory
	ros::Subscriber imu_sub   = nh.subscribe("imu",  1, &imuCallback);
	ros::Subscriber pose_sub  = nh.subscribe("odom_filtered", 1, &poseCallback);

	ros::Subscriber traj_sub  = nh.subscribe("command/trajectory", 1, &MultiDOFJointTrajectoryCallback);
	ros::Subscriber waypoint_list_sub_ = nh.subscribe("waypoint_list", 1, &WaypointListCallback);

	current_index = 0;
  const float DEG_2_RAD = M_PI / 180.0;

	// Outputs: some platforms want linear velocity (ardrone), others rollpitchyawratethrust (firefly)
	// and the current trajectory
	ros::Publisher rpyrt_command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 1);
	ros::Publisher vel_command_pub   = nh.advertise<geometry_msgs::Twist>("command/velocity", 1);
	trajectory_pub = nh.advertise<nav_msgs::Path>("current_trajectory", 1);

	double cog_z = 0.0;
	bool filtering = false;
	bool integral_window_enable;
	bool enable_ros_info;

	nh_params.param("gravity_compensation", gravity_compensation, 0.0);
	nh_params.param("x_kp", x_kp, 0.0);
	nh_params.param("x_ki", x_ki, 0.0);
	nh_params.param("x_kd", x_kd, 0.0);
	nh_params.param("x_integral_limit", x_integral_limit, 0.0);
  //nh_params.param("x_integral_window_size",x_integral_window_size,0.0);

	nh_params.param("y_kp", y_kp, 0.0);
	nh_params.param("y_ki", y_ki, 0.0);
	nh_params.param("y_kd", y_kd, 0.0);
	nh_params.param("y_integral_limit", y_integral_limit, 0.0);
  //nh_params.param("y_integral_window_size",y_integral_window_size,0.0);

	nh_params.param("z_kp", z_kp, 0.0);
	nh_params.param("z_ki", z_ki, 0.0);
	nh_params.param("z_kd", z_kd, 0.0);
	nh_params.param("z_integral_limit", z_integral_limit, 0.0);
	//nh_params.param("z_integral_window_size",z_integral_window_size,0.0);

	nh_params.param("yaw_kp", yaw_kp, 0.0);
	nh_params.param("yaw_ki", yaw_ki, 0.0);
	nh_params.param("yaw_kd", yaw_kd, 0.0);
	nh_params.param("yaw_integral_limit", yaw_integral_limit, 0.0);
	//nh_params.param("yaw_integral_window_size",yaw_integral_window_size,0.0);

	nh_params.param("x_vel_limit", x_vel_limit, 0.0);
	nh_params.param("y_vel_limit", y_vel_limit, 0.0);
	nh_params.param("z_vel_limit", z_vel_limit, 0.0);
	nh_params.param("yaw_vel_limit", yaw_vel_limit, 0.0);


    ROS_INFO("Initializing controller ... ");
	/*
	*
	*
		Initialize here your controllers
		% set initial gains and the remaining parameters from the controller.yaml
		UNKNOWN
	*
	*
	*/
  integral_error[0]=0;
	integral_error[1]=0;
	integral_error[2]=0;
	integral_error_yaw=0;

	maxXVel = x_vel_limit;
	maxYVel = y_vel_limit;
	maxZVel = z_vel_limit;
	maxYawVel = yaw_vel_limit;


	// Start the dynamic_reconfigure server
	dynamic_reconfigure::Server<rotors_exercise::ControllerConfig> server;
	dynamic_reconfigure::Server<rotors_exercise::ControllerConfig>::CallbackType f;
  	f = boost::bind(&reconfigure_callback, _1, _2);
  	server.setCallback(f);

	ros::Timer timer;
	timer = nh.createTimer(ros::Duration(0.2), timerCallback);  //Timer for debugging

	// Run the control loop and Fly to x=3.50m y=3.50m z=1m
	ROS_INFO("Going to starting position [3.50,3.50,1] ...");
	setpoint_pos = tf::Vector3(3.5,3.5,1.0);
	setpoint_yaw = 0;

	latest_pose_update_time = ros::Time::now();

	while(ros::ok())
	{
		ros::spinOnce();

		delta_time_pose = (latest_pose.header.stamp - latest_pose_update_time).toSec() ;

		// Check if pose/imu/state data was received
		if (
			(latest_pose.header.stamp.nsec > 0.0)
			&&
			((latest_pose.header.stamp - latest_pose_update_time).toSec() > 0.0)
		   )
		{
			latest_pose_update_time = latest_pose.header.stamp;

			//compute distance to next waypoint
			double distance = sqrt((setpoint_pos[0]-latest_pose.pose.pose.position.x) * (setpoint_pos[0]-latest_pose.pose.pose.position.x) +
							  (setpoint_pos[1]-latest_pose.pose.pose.position.y) * (setpoint_pos[1]-latest_pose.pose.pose.position.y) +
							  (setpoint_pos[2]-latest_pose.pose.pose.position.z) * (setpoint_pos[2]-latest_pose.pose.pose.position.z) );
			if (distance < 0.2)
			{
				prev_setpoint_pos[0]=setpoint_pos[0];
				prev_setpoint_pos[1]=setpoint_pos[1];

				//there is still waypoints
				if (current_index < latest_trajectory.poses.size())
				{
					ROS_INFO("Waypoint achieved! Moving to next waypoint");
					geometry_msgs::PoseStamped wp;
    				wp = latest_trajectory.poses[current_index];
    				setpoint_pos[0]=wp.pose.position.x;
					  setpoint_pos[1]=wp.pose.position.y;
					  setpoint_pos[2]=wp.pose.position.z;
						setpoint_yaw = atan(setpoint_pos[1]/setpoint_pos[0]);
					  current_index++;

				}else if  (current_index == latest_trajectory.poses.size()) // print once waypoint achieved
				{
					ROS_INFO("Waypoint achieved! No more waypoints. Hovering");
					current_index++;
				}
			}

			/* BEGIN: Run your position loop
				- compute your error in position and yaw
				- run the update of the PID loop to obtain the desired velocities
				 */

				 // Saving previous error
					previous_error_pos[0] = error_pos[0];
					previous_error_pos[1] = error_pos[1];
					previous_error_pos[2] = error_pos[2];
					previous_error_yaw = error_yaw;

				// Proportional error (position)
				 	error_pos[0]=setpoint_pos[0]-latest_pose.pose.pose.position.x;
	  		 	error_pos[1]=setpoint_pos[1]-latest_pose.pose.pose.position.y;
	  		 	error_pos[2]=setpoint_pos[2]-latest_pose.pose.pose.position.z;
	  		 	error_yaw = setpoint_yaw-tf::getYaw(latest_pose.pose.pose.orientation);

				//	For 0 -> 179 angle rad positive for 180 -> 359 angle rad negative
				// if 3.14 < error <-3.14 turn backwards
				if (error_yaw < -M_PI){
													error_yaw = -M_PI - error_yaw;;
													}

				if (error_yaw > M_PI){
												error_yaw = M_PI - error_yaw;
												}

				else {
									error_yaw = error_yaw;
								}

				// Derivative error
				d_error[0] = (error_pos[0]-previous_error_pos[0])/delta_time_pose;
				d_error[1] = (error_pos[1]-previous_error_pos[1])/delta_time_pose;
				d_error[2] = (error_pos[2]-previous_error_pos[2])/delta_time_pose;
				d_error_yaw = (error_yaw-previous_error_yaw)/delta_time_pose;

				// Error integral
					integral_error[0]=error_pos[0]*delta_time_pose+integral_error[0];
					integral_error[1]=error_pos[1]*delta_time_pose+integral_error[1];
					integral_error[2]=error_pos[2]*delta_time_pose+integral_error[2];
					integral_error_yaw=error_yaw*delta_time_pose+integral_error_yaw;

				//Saturate integral error
					if( integral_error[0] > x_integral_limit){
						integral_error[0]=x_integral_limit;
					}
					if(integral_error[0]< -x_integral_limit){
						integral_error[0]=-x_integral_limit;
					}
					if(	integral_error[1] > y_integral_limit){
						integral_error[1]=y_integral_limit;
					}
					if(integral_error[1]< -y_integral_limit){
						integral_error[1]=-y_integral_limit;
					}
					if(	integral_error[2] > z_integral_limit){
						integral_error[2]=z_integral_limit;
					}
					if(	integral_error[2] < -z_integral_limit){
						integral_error[2]=-z_integral_limit;
					}
					if(	integral_error_yaw > yaw_integral_limit){
						integral_error_yaw=yaw_integral_limit;
					}
					if(	integral_error_yaw < -yaw_integral_limit){
						integral_error_yaw=-yaw_integral_limit;
					}

				//PDI output
					command_pos[0]= x_kp*error_pos[0]+x_kd*d_error[0] + x_ki*integral_error[0];
					command_pos[1]= y_kp*error_pos[1]+y_kd*d_error[1] + y_ki*integral_error[1];
					command_pos[2] = z_kp*error_pos[2]+z_kd*d_error[2] + z_ki*integral_error[2];
					command_yaw = yaw_kp*error_yaw + yaw_kd*d_error_yaw + yaw_ki*integral_error_yaw;


			// rotate velocities to align them with the body frame
			// convert from local to body coordinates (ignore Z)
			tf::Vector3 vector3 (command_pos[0] , command_pos[1] , 0.0);
			vector3 = rotateZ (vector3, -tf::getYaw(latest_pose.pose.pose.orientation));

			// your desired velocities should be stored in
			x_raw_vel_cmd = vector3[0];
			y_raw_vel_cmd = vector3[1];
			z_raw_vel_cmd = command_pos[2];
			yaw_raw_vel_cmd = command_yaw;

			//Saturate  the velocities
			x_vel_cmd  = (x_raw_vel_cmd > maxXVel)   ? maxXVel  : ((x_raw_vel_cmd < -maxXVel)  ? -maxXVel  : x_raw_vel_cmd);
			y_vel_cmd  = (y_raw_vel_cmd > maxYVel)   ? maxYVel  : ((y_raw_vel_cmd < -maxYVel)  ? -maxYVel  : y_raw_vel_cmd);
			z_vel_cmd  = (z_raw_vel_cmd > maxZVel)   ? maxZVel  : ((z_raw_vel_cmd < -maxZVel)  ? -maxZVel  : z_raw_vel_cmd);
			yaw_vel_cmd  = (yaw_raw_vel_cmd > maxYawVel)   ? maxYawVel  : ((yaw_raw_vel_cmd < -maxYawVel)  ? -maxYawVel  : yaw_raw_vel_cmd);

			/* A)
			 * Some platforms receive linear velocities in body frame. lets publish it as twist
			 */
			geometry_msgs::Twist velocity_cmd;

			velocity_cmd.linear.x = x_vel_cmd;
			velocity_cmd.linear.y = y_vel_cmd;
			velocity_cmd.linear.z = z_vel_cmd;

			velocity_cmd.angular.x = 0.03; // this is a hack for Ardrone, it ignores 0 vel messages, but setting it in these fields that are ignored helps to set a real 0 vel cmd
			velocity_cmd.angular.y = 0.05;
			velocity_cmd.angular.z = yaw_vel_cmd;

			vel_command_pub.publish(velocity_cmd);

			/* B)
			 * Some platforms receive instead Roll, Pitch YawRate and Thrust. We need to compute them.
			 */

			// Map velocities in x and y directly to roll, pitch  is equivalent to a P controller with Kp=1
			// but you still need to compute thrust yourselves

			//Extract vertical acceleration to compensate for gravity
			tf::Quaternion orientation;
			quaternionMsgToTF(latest_pose.pose.pose.orientation, orientation);
			tf::Transform imu_tf = tf::Transform(orientation, tf::Vector3(0,0,0));
			tf::Vector3 imu_accel(latest_imu.linear_acceleration.x,
									latest_imu.linear_acceleration.y,
									latest_imu.linear_acceleration.z);
			body_accel = imu_tf*imu_accel;

			float thrust =  z_vel_cmd + gravity_compensation - (body_accel[2]-gravity);


			// Send to the attitude controller:
			// roll angle [rad], pitch angle  [rad], thrust [N][rad/s]
			mav_msgs::RollPitchYawrateThrust msg;

			msg.header 	  = latest_pose.header; // use the latest information you have.
			msg.pitch 	  = x_vel_cmd;
			msg.roll 	  =  -y_vel_cmd;
			msg.thrust.z  = thrust;
			msg.yaw_rate  = yaw_vel_cmd;
			rpyrt_command_pub.publish(msg);

		}

		ros::Duration(control_loop_period/2.).sleep(); // may be set slower.
	}
	return 0;
}
