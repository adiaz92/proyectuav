#include "mission_planner.h"
#include <ros/ros.h>

int main(int argc,char **argv)
{
  ros::init(argc, argv, "mission_planner");

  TfManagerNode tfManager;

  ros::Rate loopRate(20);

  while( ros::ok() )
  {
    ros::spinOnce();

    loopRate.sleep();
  }

  return 0;
}
