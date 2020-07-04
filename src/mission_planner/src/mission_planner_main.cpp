#include <mission_planner.h>
#include <ros/ros.h>

int main(int argc,char **argv)
{
  ros::init(argc, argv, "mission_planner");

  Mission mission;

  ros::Rate loopRate(20);

  while( ros::ok() )
  {
    ros::spinOnce();

    mission.do_mission();

    loopRate.sleep();
  }

  return 0;
}
