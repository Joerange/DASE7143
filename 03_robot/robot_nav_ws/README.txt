2026-11-16 更新：图形桌面执行 cd ~/robot_nav_ws && ./scripts/start_robot.sh。
自动启动 CAN、底盘和运动许可准备，只有任务开始需要手动点击。
关闭受管理窗口或 Ctrl+C 会停止任务/底盘并退出所有本应用后台；SSH完整退出用
./scripts/stop_robot.sh --all。详细行为以 CONSOLE.txt 的会话说明为准。
本轮按要求未运行测试；以下历史验证记录不覆盖本轮控制消息/会话管理修改。
原地搜索已关闭预测扫掠检查；操作员开始前确认旋转空间无人、无障碍。
导航/停靠避障、传感器/TF超时停车及独立50Hz速度保护继续生效。
最新调整：导航接管后，普通地图更新交给 Nav2 持续重规划，不再由任务层反复取消目标。
雷达帧过期阈值为2.5秒，原始点云处理期限为1.5秒；任务开始仍由操作员手动点击。
最新停靠策略：车头到可见垃圾桶表面的目标净距为10cm，软件到达区间为8–12cm。
首次确认与RGB-D定位、初始路径通过后开始靠近；行驶中根据同一垃圾桶的YOLO+RGB-D
连续观测平滑更新目标，最多每秒一次。深度桶面法向可靠时沿该法向停靠，否则朝向桶面中心。
目标离开画面、深度无效或画面边缘裁切时保留上次目标，不撤销全局运动许可。
导航线速度上限0.25m/s，角速度上限0.30rad/s；近距及最后45cm上限0.1m/s，到点前按制动距离继续减速。
现场通道由操作员确认畅通；导航避障交给Nav2，关闭独立保护节点的额外导航预测扫掠。
桶前误报修复：点云投影前过滤缺少邻域支持且未连续出现的孤立点；稀疏真实返回可通过
相邻帧里程计坐标匹配保留。SLAM仍订阅/scan，Nav2直接使用/perception/lidar_filtered点云。
无回波或被滤掉的点不伪造为自由空间；不删除垃圾桶或清空整条路径。新启动会建立新SLAM地图。
RGB-D推理线程最多等待原采集时刻TF 150ms，仍遵守0.5秒帧龄限制。界面与事件记录显示
停靠点占用/未知栅格、路径失败和规划超时的具体原因。本轮仅构建部署，未运行测试。

Bunker Mini 垃圾桶搜索与停靠：在线 SLAM + 本地控制台（2026-11-09）

本机源码：/Users/joe/HKU/DASE7143/robot_nav_ws
Jetson 工作区：/home/dase-orinnx/robot_nav_ws
~/ros2_ws 提供现有底盘/传感器依赖；本工作区作为 ROS 2 Humble overlay。
接口与安全契约见 CONTRACT.md；桌面操作见 CONSOLE.txt。
本轮测试证据与验收范围见 ../robot_console_20261109/VALIDATION.txt。

主任务行为
  人工放置或驾驶到起始点后，再启动自主任务。无需已有地图、map.yaml、AMCL 初始位姿
  或完整场地建图。enable_navigation=true 启动唯一 SLAM Toolbox 和现有 Nav2；
  SLAM 订阅 /scan，发布 /map、map→odom 及真实匹配输出 /slam_toolbox/pose；
  真实底盘提供 /odom 和 odom→base_link。主任务不再提供固定地图启动分支。

  人工授予运行时运动权限并点击“开始任务”后，等待初始有效地图、定位、感知、
  独立保护节点和 Nav2 就绪，再原地低速连续搜索。YOLO-World 稳定候选触发单批
  最多6个候选的异步确认，同时计算 RGB-D 定位与停靠路径。
  一旦身份确认、当前深度定位、实际停稳和已知自由空间路径同时通过，立即转入靠近；
  无需转满一圈、等整场建图完成或保存地图。已确认垃圾桶但路径尚未可靠时进入
  WAIT_PATH，保持停车并等待补充观测/重规划，不继续旋转或驶入未知区域。

  行驶中 SLAM 持续更新地图，Nav2 保留实时障碍层和原车体轮廓。导航前复核停靠点和
  初始路径；导航接管后，普通地图栅格、尺寸及原点变化不再触发任务层整段路径作废，
  由 Nav2 持续重规划和局部避障。SLAM 重置、坐标系变化或显著定位跳变仍停止导航，
  等待取消终态、实际停稳、重新观测和路径验证，并更新几何代次和
  任务 UUID，丢弃旧证据并要求重新确认目标；恢复有期限，失败进入 HOLD。
  必需的雷达/定位/底盘通信/控制服务失效会撤销运行时权限，恢复后须重新人工授权并开始。
  相机目标丢失或深度无效不再撤销权限；只有连续有效的同目标RGB-D观测才能更新停靠点。

  近距阶段前往最新被Nav2接受的视觉修正停靠点；没有新观测时沿用上一有效点。
  到达要求停靠点误差≤0.02m、朝向误差≤0.10rad、地图估计净距0.08–0.12m、
  导航取消已结束，且真实里程计连续静止≥0.7s。若最终没有有效RGB-D复核，界面只报告
  “已到停靠点并停稳；实时净距未复核”，不把地图估计当作实测净距验收。
  车头参考 base_link 的 x=0.345m；独立最小净距阈值6cm，不自动倒车。
  Option B 自动探索未知区域、跨视点探索记忆和独立末端控制器不属于本次实现。

五个 ROS 包
  robot_perception：同步 RGB-D、观测时刻 TF、YOLO-World、实例关联、候选批量确认、
    深度表面定位、关键帧留存。VLM 只提供候选语义结论，不决定运动。
  robot_mission：搜索/确认/停稳/已知自由空间规划、地图重验证、导航与近距停车状态机。
  robot_safety：独立50Hz最终速度保护、原始命令新鲜度、车体制动扫掠、SLAM 健康检查、
    运行时授权锁存与控制服务/底盘通信看门狗。
  robot_bringup：保留传感器外参、车体过滤与 /scan、在线 SLAM、Nav2、RViz 启动配置。
  robot_console：原生桌面四视图、独立普通用户后台、实际状态回读、操作记录与日志导出。

构建（Jetson）
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  cd ~/robot_nav_ws
  colcon build --symlink-install --packages-select robot_perception robot_mission robot_safety robot_bringup robot_console
  source install/setup.bash

默认静止监视入口
  export ROS_DOMAIN_ID=180 ROS_LOCALHOST_ONLY=1
  ros2 launch robot_bringup task.launch.py start_sensors:=true enable_navigation:=true rviz:=true

  未传入 map，也不启动 AMCL/map_server。启动文件不配置 CAN、不启动底盘、不自动开始任务。
  默认 allow_runtime_control=false：保护节点只在 /cmd_vel_guard_preview 发布零速度，
  不创建 /cmd_vel 发布者；/mission/start 和授权 true 请求均被拒绝。
  无真实里程计时，SLAM/安全状态等待或报告 odom_stale 是正常结果；不要向生产域注入
  测试里程计、静态 map→odom 或伪造 /amcl_pose 来绕过准备条件。
  enable_navigation=false 可单独查看传感器/感知；此时没有在线地图与 Nav2。
  不要重复启动已在运行的相机、雷达或 SLAM 节点。

原生桌面控制台
  cd ~/robot_nav_ws
  ./scripts/run_console.sh monitor --fullscreen
  ./scripts/run_console.sh --demo --fullscreen

  第一条连接已有本地后台，或启动只读监视后台；默认域180、仅 localhost。
  任务、独立保护和受控后台必须使用相同 ROS 域。更改窗口环境不会改变已运行后台的域。
  第二条是标记为 SIMULATED 的离线演示，不使用 ROS、控制连接或 CAN。
  RGB 检测框绑定原图时间戳；深度 Z 与车头净距分开显示；地图按当前原点/分辨率投影。
  CAN UP 不等于底盘通信正常，未知/过期速度不会显示为0。
  关闭窗口与 Esc 不发送停止命令；停止应使用红色“停止任务”，并检查后台回读。
  完整按钮、部署、权限边界、日志导出及断线行为见 CONSOLE.txt 和
  console_service/HELPER_PROTOCOL.txt。生成部署文件不表示已安装或已启动服务。

获得明确实车许可后的运行时控制流程（操作说明，不代表已执行）
  任务启动仍保持默认禁用运动，仅提供运行时授权能力：
    export ROS_DOMAIN_ID=180 ROS_LOCALHOST_ONLY=1
    ros2 launch robot_bringup task.launch.py start_sensors:=true enable_navigation:=true \
      allow_runtime_control:=true rviz:=true

  allow_runtime_control=true 只让独立保护节点拥有最终 /cmd_vel 出口；它立即以50Hz输出零，
  初始 authorized=false，不从参数、磁盘、上次任务或重启状态恢复权限。
  旧 motion_enabled:=true、start_chassis:=true 入口已废弃并拒绝；不要用启动参数授权运动。
  autostart 保持 false；CAN、底盘、运动授权和开始任务是不同操作。

  受控服务须先按 console_service/HELPER_PROTOCOL.txt 单独审阅和部署。默认服务使用域180，
  后台进程独立于窗口渲染，发布 /console/control_heartbeat 和 /chassis/health，
  两者须有同一 backend session_id 且均新鲜（默认≤0.5s）。只读监视后台不发布控制租约。
  “启用 CAN”只配置固定 can0/500000；“启动底盘驱动”另行执行并核验实时被动 CAN 反馈、
  /bunker_status、真实里程计及驱动进程。CAN=UP 单独不能授权。

  等初始地图、SLAM、传感器、实际静止与 Nav2 就绪后，人工“允许任务运动”，再“开始任务”。
  对应 ROS 服务（必须在同一 ROS 域，且受控后台已满足上述条件）：
    ros2 service call /safety/set_motion_enabled std_srvs/srv/SetBool '{data: true}'
    ros2 service call /mission/start std_srvs/srv/Trigger '{}'
  不满足任一条件时返回具体拒绝原因；服务请求成功还须核对 /safety/status、/mission/state。
  开始任务产生新 UUID，在新代次安全回读就绪前保持 INIT/source=none。

  独立撤销与任务停止接口：
    ros2 service call /safety/set_motion_enabled std_srvs/srv/SetBool '{data: false}'
    ros2 service call /mission/stop std_srvs/srv/Trigger '{}'
  false 请求不依赖相机、TF、地图或云网络就绪，立即清空命令并输出零，同时异步请求任务停止。
  /mission/stop 本身将任务置 HOLD、撤销当前来源并取消导航；它不单独改变全局授权锁存。
  控制台停止按钮同时请求两者，并等待授权撤销、零输出、任务 inactive、导航终态和实测静止。
  保护节点/后台重启、租约失效、底盘通信或健康条件失败均不会自动重新授权或重启任务。
  停止驱动/关闭 CAN 还要求持续实时 CAN 静止证据，并只管理后台拥有的驱动；无法确认时
  保持操作失败/未知，不能把关闭 CAN 当作已经停稳。独立硬件失联停车仍需现场验证。

SLAM 健康与导航边界
  安全节点检查有效且新鲜的 /map、/scan、深度、/odom、动态 map→odom→base_link 连通性
  及 TF 原始时间戳，同时要求实际 SLAM 匹配 pose。仅查到缓存旧 TF 不表示定位就绪。
  静止时 SLAM pose 可以保持，但仅当当前真实里程计仍在最近匹配 pose 对应位置附近；
  持续移动却没有匹配进展会拒绝运动。不会伪造 AMCL 消息或协方差。
  全局代价地图订阅持续更新的 /map 和更新信息，未知空间不可规划；导航前任务使用
  带朝向的矩形车体及5cm余量复核停靠点/路径，替代原车体外接圆。实时障碍层仍参与避障。
  搜索在首次可达目标就绪时结束；完整一圈有效观测而无可接受目标会停止，不自动探索。

API 与日志
  在感知/确认进程启动环境安全注入 DASHSCOPE_API_KEY 或 QWEN_API_KEY，不写入源码或日志。
  QWEN_BASE_URL 可覆盖地域端点，默认 https://dashscope.aliyuncs.com/compatible-mode/v1。
  模型 qwen3.8-flash，enable_thinking=false；仅候选事件触发请求，空场景零请求。
  单次确认期限8s；语义证据有期限，旧响应不能跨 task_generation/geometry_epoch 生效。
  任务事件：~/.ros/robot_mission；感知关键帧：~/robot_nav_ws/mission_logs/perception；
  控制台导出：~/robot_nav_ws/console_exports。导出不包含密钥、原图、API正文或提示词。

独立建图入口
  ros2 launch robot_bringup mapping.launch.py start_sensors:=true
  该入口仅用于独立建图，不与任务内 SLAM 同时运行，不启动底盘或自动任务。
  如需人工采图，应另行获得运动许可并使用已有受保护控制方案；保存地图不是主任务前置条件。

验证分层
  1. 本地软件回归：
    cd ~/robot_nav_ws
    PYTHONPATH=src/robot_perception:src/robot_mission:src/robot_safety:src/robot_console \
      python3 -m unittest discover -s tests -v
    QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests -p 'test_console_ui*.py' -v
    纯逻辑测试覆盖无 map 启动配置、地图扩展不误停、路径失效/定位跳变撤销当前来源、
    雷达/TF超时零速、运行时租约与重启、手动开始、停止证据以及控制台状态回读。
    ROS/Qt 依赖可用性和跳过项目须按实际报告记录，不将跳过算作通过。

  2. 隔离 ROS 域静止集成（需先 source 三层 ROS 环境）：
    python3 scripts/verify_online_slam.py
    使用域179，实时雷达/相机、实际 SLAM /map 与 map→odom；先验证缺里程计时拒绝，
    再注入明确标记 TEST-ONLY 的零里程计及 odom→base_link，仅用于静止软件集成。
    仅规划器/BT激活；控制器、行为服务器、平滑器保持 inactive，只请求 ComputePathToPose。
    没有 NavigateToPose 运动目标、运行时授权、底盘启动或 CAN 配置操作；还检查只读控制台。
    默认拒绝已 UP 的 CAN 和任何已运行底盘驱动。若操作员已核实 can0 的既有 UP 状态，
    且无底盘驱动，可选择只读兼容入口：
      python3 scripts/verify_online_slam.py --allow-preexisting-can-up
    此选项保留并核验原有 CAN 配置，不启用/关闭 CAN，不重复既有底盘基线测试。
    verify_nav_config.py 是该在线脚本的兼容入口；verify_stationary.py 为旧域176纯传感器
    检查，仍要求 CAN DOWN。不要为运行旧脚本擅自更改现有 CAN 状态。

  3. 实车导航验收（仍须明确现场许可）：
    新场地从零建图与真实定位精度；纯旋转匹配稳定性、滑移/里程计方向；目标确认和深度误差；
    首个可达目标立即靠近、不可确认路径停车等待、地图扩展/阻塞/跳变恢复；
    真实障碍与制动距离、通信失联和最终保护进程死亡时的硬件停车；
    受控驱动启停/CAN关闭联锁、真实停止超时和10cm停靠独立尺测。
    软件测试、演示画面及静止集成都不能替代这一层。实际已完成项目、报告路径和剩余问题
    统一记录在 ../robot_console_20261109/VALIDATION.txt，不以本文启动命令声称验收通过。
