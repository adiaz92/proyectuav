#include "transform_manager.h"
#include <ros/ros.h>

int main(int argc,char **argv)
{
  ros::init(argc, argv, "transform_manager");

  TfManagerNode tfManager;

  ros::Rate loopRate(20);

  while( ros::ok() )
  {
    ros::spinOnce();

    loopRate.sleep();
  }

  return 0;
}
