robot_bringup — online SLAM task navigation on ROS 2 Humble

Task default: diagnostics only, no chassis, no mission start, no velocity authorization.
Enable online navigation without a map file:
  ros2 launch robot_bringup task.launch.py start_sensors:=true enable_navigation:=true
This starts one SLAM Toolbox and the Nav2 stack. The independent guard remains disarmed.
No AMCL or static map_server is launched. There is no fixed-map launch mode.
A real /odom and odom->base_link publisher must be supplied before SLAM can initialize.
The task waits for current map/localization/sensors/safety/Nav2 readiness and manual start.
Arming or starting a chassis requires explicit operator authorization; these commands do neither.

Launch routes:
  sensor_tf.launch.py: installation TF only; original measured/approximate transforms preserved.
  sensors.launch.py: sensor transforms, optional real camera/LiDAR, cloud self-filter and /scan.
  task.launch.py: diagnostics, optional online SLAM/Nav2, perception, task and command guard.
  nav2.launch.py: one async_slam_toolbox_node + Nav2. No map argument.
  mapping.launch.py: standalone mapping + sensors, no navigation/task/guard/chassis.
Do not run mapping.launch.py beside task/nav2; that would duplicate SLAM's map->odom ownership.

TF ownership:
  SLAM Toolbox: map->odom; chassis: odom->base_link; mounting launch: sensor installation TF;
  RealSense driver: internal camera optical frames. No fabricated AMCL pose is published.
SLAM accepts /scan, produces /map and its accepted pose on /slam_toolbox/pose.
The task guard uses localization_mode=slam. Its standalone AMCL compatibility is not a task launch route.

Online map configuration:
  global StaticLayer subscribes to repeated /map and optional /map_updates, including resizing.
  Real-time obstacle and inflation layers, the 690x570mm footprint and 50mm padding remain.
  Navfn allow_unknown=false; both costmaps track unknown; no reverse/spin recovery.
  SLAM map_file_name='' starts a new graph; map updates every1s; no mandatory map saving.
  Humble 2.6.10 stamps map->odom with latest scan acquisition time plus transform_timeout.
  restamp_tf=false keeps this behavior on newer compatible releases.
  Its outer scan gate checks translation distance; minimum_travel_distance=0 permits in-place
  rotation. minimum_time_interval=.2 bounds ingestion to5Hz; minimum_travel_heading=.025.
  Interactive graph editing is disabled to avoid accidental graph changes during a task.

Control chain:
  Nav2 controller/behavior /cmd_vel_nav -> velocity smoother /cmd_vel_nav_smoothed -> guard.
  Only the armed guard publishes /cmd_vel. Disarmed guard publishes zero preview only.
  All launch defaults remain motion_enabled=false, autostart=false and start_chassis=false.
  No launch runs an ip-link/CAN setup command.

Primary implementation references:
  https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/src/slam_toolbox_common.cpp
  https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/static_layer.cpp
