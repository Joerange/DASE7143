# Coursework 2 · Option B 隐藏目标探索

独立 ROS 2 Humble 工作空间，基于本地 robot_nav_ws 当前源码扩展。默认启用 Option B；可通过 `exploration_enabled:=false` 运行原地旋转基线。目标位置由相机感知，启动只指定 TA 选定的类别。

## 已实现

- 0.5 m 世界坐标网格、12 个朝向分区、观察历史与失败冷却；普通地图扩展保留记忆，坐标重置使记忆失效。
- 已知自由空间内选择视点；评分结合视觉新覆盖、未知区域收益与路径成本。LiDAR 已知区域也参与视觉探索，射线在障碍处截止。
- 后台计算候选路线，主控制循环继续响应；Nav2 执行新视点导航，到达后取消完成、实测静止 0.7 s，再采集新帧进行整图 VLM 检查。
- 整图 found 后复用 YOLO 候选确认与 RGB-D 定位，最终以当前目标几何验收停车。not_found 更新当前视角；uncertain/接口失败补拍一次后继续其他视角。
- 无可达视点时保持停车并定期重新选点；总任务 300 s 到期进入 HOLD，原因 search_timeout。目标不存在不计入找到目标成功率。
- 控制台地图显示观察位置/朝向、临时排除点、路径与导航目标；详情显示覆盖收益。任务日志记录请求、结果、记忆及状态。

## Jetson 构建与运行

将本目录放到 `~/robot_option_b_ws`。在新终端只加载 ROS 与传感器/底盘依赖工作区，随后构建这个 overlay：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/robot_option_b_ws
colcon build --symlink-install
source install/setup.bash
```

复用现有 YOLO 环境及 `~/yolo-world-agv/models/yolov8s-worldv2-trash.pt`，程序启动时重新设置目标词汇。服务账号沿用 `DASHSCOPE_API_KEY` / `QWEN_API_KEY` 环境变量；在启动终端注入。三个识别节点使用同一目标标签和 `qwen3.8-flash`，模型可以在 perception.yaml 中调整。

```bash
# TA 选择背包；也可使用 "trash can"、"umbrella"
./scripts/run_option_b.sh backpack
# 对照试验：相同场景、相同起点，关闭探索
./scripts/run_option_b.sh backpack exploration_enabled:=false
```

脚本启动传感器、SLAM、Nav2、任务和界面，不自动配置 CAN 或开始行驶。通过原有控制台启动底盘、允许任务运动并开始任务；这些仍使用已有的独立停车链。控制台/受控服务部署路径已指向新工作空间，具体见 `CONSOLE.txt`。两套工作空间的同名节点和服务一次运行一套。

完整桌面会话入口 `./scripts/start_robot.sh` 默认垃圾桶；TA 选择其他类别时使用上面的目标参数入口和控制台。现有服务如仍加载旧 overlay，需要先按新目录的 console_service 配置更新为新 overlay。

## 关键参数

`src/robot_bringup/config/mission.yaml`：300 s 总期限、0.15 m/s 导航上限、0.60 m 目标净距、0.55–0.65 m 验收区间。独立保护保留 LiDAR/深度/定位新鲜度、车体扫掠检查和零速保持。正常地图更新交由 Nav2；未加入逐格运动审批。`option_b.py` 的首版策略为 12 个朝向、45 s 转向/导航期限、一次补拍；探索算法参数见 `exploration.py`。

VLM 使用请求后新采集图像，结果绑定任务、几何代次和请求 ID；到期或换视点后的响应不会驱动接近。整图结果只确认类别存在，目标接近仍要求实例确认与有效深度。

## 验证和现场试验

```bash
PYTHONPATH=src/robot_perception:src/robot_mission:src/robot_safety:src/robot_console \
  python3 -m unittest discover -s tests -p test_option_b.py -v
```

本机验证范围及继承测试的基线差异见 `validation/REPORT.md`。Mac 上未执行 ROS 构建、云端识别或实车行驶。新增测试使用真实任务逻辑和替代 ROS 传输，不能作为现场试验结果。

现场至少三轮：两轮隐藏目标（不同位置/类别，加入干扰物），一轮目标不存在。有目标轮先录原地旋转基线，再恢复同一起点并重启任务/SLAM运行增强版。画面同时录制机器人与控制台记忆，停车后尺测车头前缘至目标表面净距。场景布置、TA 标签、代码版本和结果填入 `trials/template.json`。

日志默认位于 `~/.ros/robot_option_b`；停车视点图像和语义结果位于 `~/robot_option_b_ws/mission_logs/views`。提取单轮记录：

```bash
python3 scripts/summarize_trial.py ~/.ros/robot_option_b/mission_YYYYMMDD_HHMMSS.jsonl \
  --measured-gap 0.60 --output trial_result.json
```

`--measured-gap` 填实际尺测值；目标不存在时省略此参数。
