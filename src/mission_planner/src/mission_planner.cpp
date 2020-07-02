#include "mission_planner.h"

mission::missionNode()
{
  nh.getParam("parcel_dispatcher/check_pad_position", checkpad);
  nh.getParam("parcel_dispatcher/charging_pad_pose", chargingpad);
  nh.getParam("parcel_dispatcher/shelve_pose", shelve);
  nh.getParam("parcel_dispatcher/husky_pose", husky);




}
