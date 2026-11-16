静态传感器 TF 配置（不启动底盘）

部署工作区：/home/dase-orinnx/robot_nav_ws
源码包：robot_bringup
配置：src/robot_bringup/config/sensor_extrinsics.yaml

启动：
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  source ~/robot_nav_ws/install/setup.bash
  ros2 launch robot_bringup sensor_tf.launch.py

此命令只发布以下三个安装变换，不启动相机、雷达、底盘、Nav2 或里程计：
  base_link -> laser_link
  base_link -> camera_mount -> camera_link

RealSense 驱动启动后继续提供 camera_link 到彩色/深度/光学坐标系的原厂变换。
不要再用另一份 URDF/静态发布器重复发布上述相同 child frame。
关闭静态发布器用 Ctrl-C；配置文件改动后重新启动发布器。
固定传感器变换不会补出 map->odom 或 odom->base_link，不应伪造这两条动态关系。

发布者和相机驱动需要处于同一 ROS_DOMAIN_ID。仅做隔离静态检查时可在各终端统一设置：
  export ROS_DOMAIN_ID=175
  export ROS_LOCALHOST_ONLY=1
实际任务使用其自身统一 ROS 域；本次没有设置开机自启。

参考原点：base_link 为履带包络中心的地面投影；x前、y左、z上。
雷达：xyz=(0.125,0,0.600) m，yaw=+90度，按用户要求固定；roll/pitch暂取0。
相机外壳参考：xyz=(0.195,0,0.500) m，roll=0、pitch=+12度、yaw=0。
在此 ROS 约定下，正 pitch 表示光轴向下；12度来自已有深度地面拟合，用户已确认实际下倾。
camera_mount 不是 RealSense 的 camera_link。D455 名义几何下，camera_link 在外壳中心约前8.35 mm、左47.5 mm处。
相机内部光学旋转及约59 mm的彩色/深度基线由驱动提供，不再次硬编码。

尺寸来源和状态都记录在 YAML。高度50 cm、距履带前缘15 cm、中线位置是用户尺量；
外壳中心按几何中心处理。用户已要求不再补测其他尺寸，沿用现有近似角度和参考点。
碰撞几何按用户尺寸图采用长690、宽570、高335 mm；俯视图下方为车头(+X)，图右侧为车体左侧(+Y)。
物理轮廓为 x=±0.345、y=±0.285 m，Nav2 另加每侧0.05 m初始规划余量，包络为0.790×0.670 m。
nav2_footprint.yaml 同时提供 local_costmap/global_costmap 的 footprint 与 footprint_padding。
这是几何参数补充文件，后续合并入完整 Nav2 参数；不能用它替代完整导航配置。sensor_tf.launch.py不加载它。
原始轮廓未加余量；不要再把已扩大的0.790×0.670 m轮廓配合0.05 m padding重复扩大。
详细 TF 来源与验证见 robot_完整实现方案.txt 第22节，已定碰撞尺寸见第23节。

官方参考：
https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md
https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_description/urdf/_d455.urdf.xacro
