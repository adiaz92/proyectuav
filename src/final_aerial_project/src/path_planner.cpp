#include <ros/ros.h>
#include <mav_planning_msgs/PlannerService.h>
#include "aerial_project/path_planner.h"

namespace voxblox
{

    PathPlanner::PathPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
        : nh_(nh),
          nh_private_(nh_private),
          esdf_server_(nh_, nh_private_)
    {

        nh_private_.param("collision_radius", p_collision_radius_, 1.0);

        //subscribers
        odometry_sub_ = nh_.subscribe("kalman_filter/odom_filtered",1,&PathPlanner::OdometryCallback,this);
        desired_pose_sub_ = nh_.subscribe("command/pose",1,&PathPlanner::DesiredPoseCallback,this);
        //estimated_pose_sub = nh.subscribe("kalman_filter/pose_estimation", 1, &PathPlanner::PoseEstimationCallback, this);
        //Publishers
        // es necesario un pub para el mission planner¿?

        //ServiceClient
        planner_client_ = nh_.serviceClient<mav_planning_msgs::PlannerService>("voxblox_rrt_planner/plan");
        path_pub_client_ = nh_.serviceClient<std_srvs::Empty>("voxblox_rrt_planner/publish_path");

        //Fill info for call planner_srv_
        mav_planning_msgs::PlannerService plannersrv;
        plannersrv.request.start_pose = start_pose;
        plannersrv.request.goal_pose = goal_pose;
        //call service planner_srv_
        planner_client_.call(plannersrv);

        // If success mean that there is a path, then call path_pub_client_
        if (plannersrv.response.success){
          std_srvs::Empty srv;
          path_pub_client_.call(srv);
        } else {
        // Goal pose is occupied or unknown
         Eigen::Vector3d point;
         point[0]=goal_pose.pose.position.x;
         point[1]=goal_pose.pose.position.y;
         point[2]=goal_pose.pose.position.z;
          goal_free = isCollisionFree (point);
          // If the goal pose is not and obstacle find a smaller goal pose
          if (goal_free){
            ROS_INFO ("Goal pose unknown in map");
          } else {
            //The goal pose can't be reached because is a obstacle
            ROS_ERROR("Goal pose occupied!");
          }
        }

    }

// Method that checks whether a point is occupied or not
// The point is collision free if it's distance to the nearest obstacle is bigger
// than the collision radius defined
bool PathPlanner::isCollisionFree(const Eigen::Vector3d &position)
    {
        double distance = 0.0;
        const bool kInterpolate = false;
        esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(
            position, kInterpolate, &distance);

        if (distance < p_collision_radius_)
        {
            return false;
        }
        return true;
    }
void PathPlanner::OdometryCallback(nav_msgs::Odometry msg){
  start_pose.header = msg.header;
  start_pose.pose = msg.pose.pose;

}
/*void PoseEstimationCallback(const geometry_msgs::PoseStamped& pose_estimation_msg){
  start_pose.header = pose_estimation_msg.header;
  start_pose.pose = pose_estimation_msg.pose;
}*/
void PathPlanner::DesiredPoseCallback(geometry_msgs::PoseStamped msg){
  goal_pose.header = msg.header;
  goal_pose.pose = msg.pose;
}

/*void PathPlanner::GoalModification (const geometry_msgs::PoseStamped &goalpose, const geometry_msgs::PoseStamped &startpose){
   int i=0;
   goal_array[0]= goalpose;
   if (goalpose.position.x/2 > startpose.position.x){
    goal_array[i].position.x= goalpose.position.x/2;
    } else{
    goal_array[i].position.x= goalpose.position.x;
    }
   if (goalpose.position.y/2 > startpose.position.){
     goal_array[i].position.y= goalpose.position.y/2;
    } else{
     goal_array[i].position.y= goalpose.position.y;
    }

}*/



} // namespace voxblox
