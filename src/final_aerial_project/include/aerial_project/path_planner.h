#ifndef COLLISION_CHECK_H
#define COLLISION_CHECK_H

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <map>

#include <voxblox_ros/esdf_server.h>

namespace voxblox
{
    class PathPlanner
    {

    public:
        PathPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

        bool isCollisionFree(const Eigen::Vector3d &position);
        void GoalModification (const geometry_msgs::PoseStamped &goalposition);

    protected:

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        voxblox::EsdfServer esdf_server_;

        //subscribers
        ros::Subscriber odometry_sub_;
        ros::Subscriber desired_pose_sub_;
        ros::Subscriber estimated_pose_sub;

        //Publishers

        //Service
        ros::ServiceClient planner_client_;
        ros::ServiceClient path_pub_client_;

        //parameters
        double p_collision_radius_;
        geometry_msgs::PoseStamped goal_pose;
        geometry_msgs::PoseStamped start_pose;
        geometry_msgs::PoseArray goal_array;
        bool goal_free;

        //Callbacks
        void OdometryCallback(nav_msgs::Odometry msg);
        void DesiredPoseCallback(geometry_msgs::PoseStamped msg);
        //  void PoseEstimationCallback(const geometry_msgs::PoseStamped& pose_estimation_msg);
    };
} // namespace voxblox
#endif
