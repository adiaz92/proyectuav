# uavproyect

## Description:

The framework used is ROS (Melodic), Gazebo as simulator and RVIZ as visualizator.

The robotic application to solve takes place in an automatized industrial warehouse environment were AGVs are in charge of transporting dispatched parcels to the loading bay area. An UAV (a.k.a drone) is used to continuously check the inventory of products stored in the shells with the aim of avoiding mistakes in the prepared orders.  Whenever a missing product or a prone to error reference is detected, the drone should get information of a centralized system about the AGV who has the parcel and find it.

### Mission to solve

Departing  from  the  charging  station,  the  drone  has  to  obey  the  requests  published  in  the /parcel_dispatcher/product_request topic. 
This topic pipes a message that includes the approximated solution of the target, its location (SHELVE, HUSKY or END) and the id of a marker (4x4_50 aruco dictionary of 25cm side) identifying the object that the drone may find. 

In this case the drone will be requested to locate two different parcels making a vertical search in a shelves, followed by a horizontal search to locate a marker over the husky and finally return to the charging pad.

In  order  to  register  the  located  items,  the  drone  may  fly  to  the  check  pad  and  publish  a productFeedback msg to the /firefly/product_feedback topic, with the id of the identified parcel and its location. The parcel dispatching is managed by the dispatcher_station_nodethat will deliver parcel queries in sequential order. 

In order to register the location of an identified parcel, the drone has to be inside an 1m radius of the check pad and publish a productFeedback msg to the /firefly/product_feedback topic, with  the  id  of  the  identified  parcel  and  its  location.  The  marker_id  message  inside  the  Feedback message has to match with the one broadcasted in the /parcel_dispatcher/product_request topic.

## Gazebo base scenario:

An AscTec firefly drone is the flying platform which is equipped with the following sensors:

• Visual-Inertial (VI) sensor:
  
  ◦ A pair of visual cameras
   
   ◦ An IMU
  
  ◦ A depth camera

• A down-looking visual camera

• A fake odometry sensor

• An additional IMU

131  markers  are  situated  on  the  ground.  These  are  derived  from  a 5x5_250  aruco dictionaryand have a size of 15 cm side. 

## Package distribution

### State estimation: 

### MAV control:

The  control  of  the  MAV  is  divided  in  a  high-level  controller  (position  controller)  and  a  low-level controller (attitude controller):  

•The low-level controller is implemented within the rotorS simulator. 

•The high-level controller is implemented with a PID controller of each input (x,y,z and yaw) and convert them to pitch,roll, yaw_rate and thrust commands which are then sent to the low level, attitude controller.

It subscribes to topic /odom_filtered from the state estimation to now the current position and to the /waypoint_list to recibe the commands of x,y,z and yaw.

### Path Planning:

For planninng the library Voxblox is used to create the map. It is divided in 3 packages: Voxblox, mav_voxblox_planning and final_aerial_project.

The voxblox node ingests pointcloud data and produces both a TSDF and an ESDF.

For planning, we have use the rrt_planner which subscribes to the latest ESDF layer and creates a path that is send to the controller. The main problem with this node is that the map must been known since the planner need it to make the trajectory. As the map is unknow at the beginnig we only can move in the zone that firefly is seeing, any other point will be indicated as occupied by the planner.

For the interaction between the mission planner and the path_planner the node path_planner has been made, this node subscribes the commands from the mission_planner node and call the service "path" of the rrt_planner, if the trajectory can be made it call the service "path_publish" to publish the path to the position controller. 

The part that it has to be implemented yet is the one with the path service doesn't make the trajectory, so the path_planner node, has to give an alternative point in order to explore the map and advance to the desired point.

### Mission Planner:
