# gpsins_localizer

This ROS package is for providing a localization solution that relies only on
GPS-INS data. An example of a GPS-INS system that provides this data is the
[Novatel SPAN](https://www.novatel.com/products/span-gnss-inertial-systems/)
system. Currently, this package supports two Novatel drivers:
[`novatel_gps_driver`](https://github.com/swri-robotics/novatel_gps_driver) and
[`novatel_oem7_driver`](https://github.com/novatel/novatel_oem7_driver).
However, it should be easy enough to add support for additional
GPS drivers.

## Dependencies

- [GeogrpahicLib](https://sourceforge.net/projects/geographiclib/). Can be installed via apt (`libgeographic-dev`) or from rosdep (`geographiclib`).
- [novatel_gps_msgs](https://github.com/swri-robotics/novatel_gps_driver/blob/master/novatel_gps_msgs). Can be installed via apt (`ros-$ROS_DISTRO-novatel-gps-msgs`) or rosdep (`novatel_gps_msgs`).
- [novatel_oem7_msgs](https://github.com/novatel/novatel_oem7_driver/tree/master/src/novatel_oem7_msgs). Can be installed via apt (`ros-$ROS_DISTRO-novatel-gps-msgs`) or rosdep (`novatel_oem7_msgs`).

## ROS API

#### Subs

- `imu`([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))  
This topic is used to populate the angular velocities of the vehicle state.
- `gps/inspva` ([novatel_gps_msgs/Inspva](https://github.com/swri-robotics/novatel_gps_driver/blob/master/novatel_gps_msgs/msg/Inspva.msg)) OR  
`novatel/oem7/inspva` ([novatel_oem7_msgs/INSPVA](https://github.com/novatel/novatel_oem7_driver/blob/master/src/novatel_oem7_msgs/msg/INSPVA.msg))  
This topic is used to calculate vehicle Pose in the map frame, as well as a forward velocity estimate.
- `gps/bestpos` ([novatel_gps_msgs/NovatelPosition](https://github.com/swri-robotics/novatel_gps_driver/blob/master/novatel_gps_msgs/msg/NovatelPosition.msg)) OR  
`novatel/oem7/bestpos` ([novatel_oem7_msgs/BESTPOS](https://github.com/novatel/novatel_oem7_driver/blob/master/src/novatel_oem7_msgs/msg/BESTPOS.msg))  
This topic is only used if both the `msl_height` and `mgrs_mode` parameters are true.
It is used to receive undulation data for height calculations.

#### Pubs

- `current_pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
For use with autoware
- `current_velocity` ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))  
For use with autoware

#### Transform Listeners

- `earth` -> `map` static TF [optional]  
This tf is required in order to transform GPS coordinates into an existing map frame. If the map frame doesn't already exist, set the `create_map_frame` param to `true`.
- `base_link` -> `gps` static TF  
Required to calculate the offset between the gps-ins sensor mount location and the `base_link`.

#### Transform Broadcasters

- `map` -> `base_link`  
The updated pose of the vehicle
- `earth` -> `gps_measured` [optional]  
The pose of the gps-ins sensor in the earth frame.

#### Configuration Parameters

See the `config/params.yaml` file for a list of parameters and their descriptions.

## Notes

- This package assumes that the imu is mounted close enough to the base_link so that the angular velocity measurements from the imu are considered the angular velocities of the base_link.
- It is assumed that the gps-ins roll/pitch orientation data follows a y-forward, z-up coordinate frame as shown [here](https://docs.novatel.com/OEM7/Content/Resources/Images/Vehicle%20Body%20Frame%20Airplane_372x378.png).
