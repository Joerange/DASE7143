robot_perception — 主任务感知与语义确认

入口：perception_node（YOLO venv 解释器）、vlm_confirm_node（系统 Python）。
必须用同一 ROS_DOMAIN_ID 与 clock。主程序由 robot_bringup 启动。

perception_node 使用对齐 RGB/深度/CameraInfo 的近似同步；回调仅保存最新一组消息。
推理和网络为不同节点/工作器。处理后输出保留原始采集时间，未处理旧帧会被替换。
完整 TF 使用采集时间查询。map TF 缺失时可继续本地识别和输出 base_point，但 target.valid=false。
无 map→odom/真实 odom 时不得将静止感知输出误当作已具备导航定位。

初始类别：trash can，加 person/chair/potted plant/bottle/backpack 作为干扰类别。
set_classes 启动时完成，预热使用空本地图像，不调用云 API。
只有 trash can 稳定轨迹（最近3帧至少2次>=0.30）可触发；维持门限0.20。
同一时刻最多一个确认事件；短批次窗口0.2秒，最多6个候选，整图/裁剪来自同一帧。
在 SEARCH_CONTINUOUS，或 REOBSERVE_RECOVERY 且 reconfirm_required=true 时发送新事件。
普通地图变化或路径暂时受阻时保留实例确认，WAIT_PATH 不自动发起新请求；INIT/HOLD 仅输出诊断。
不通过定时器、空场景或者没有目标的整图请求 VLM。

VLM 默认 qwen3.8-flash，enable_thinking=false，JSON object 输出。
读取 DASHSCOPE_API_KEY 或 QWEN_API_KEY；可用 QWEN_BASE_URL 覆盖地域 endpoint。
不从旧 simple_vlm YAML 拷贝或打印密钥。不配置 Key 时明确返回 missing_api_key。
必须每个请求编号恰好一次返回 match/no_match/uncertain；遗漏、重复、未知ID、额外字段
或未完成回答均无效。8秒硬期限；迟到结果不采纳，任务代次或 geometry_epoch 不一致不发布。
没有自动无限重试。拒绝实例按地图位置冷却15秒；uncertain 只有图像证据改善后有界重试。
若SLAM重置时旧HTTP工作器仍占用，worker_busy 表示新请求尚未发送，按剩余占用时间冷却后重试；
它不被缓存成语义拒绝。总体重观察和任务期限由任务节点约束。

SLAM重置或显著定位跳变时，任务节点更新 task_generation 和 geometry_epoch。
感知节点清除旧地图中的实例和拒绝缓存，并从新上下文开始时间之后采集的RGB-D重新建立轨迹。
旧帧即使在重置后才到达，也不能获得新上下文标签；已完成但尚未发布的推理结果、候选事件和
迟到HTTP结果同样检查双重上下文。普通地图扩展不改变该上下文，不使已有身份自动失效。

深度采用框内多区域支持，按编码转换米，处理 CameraInfo 畸变，排除地面/无效值，
拒绝多簇混合、空间覆盖不足、过大离散。它估计可见表面，不是物体中心。
车头净距=base_point.x-0.345。当前默认质量门限需要使用真实场景评估。
确认身份的地图记录可在短暂离开视野后重新关联，但每次接近必须有新深度。
身份歧义时不把旧ID随意绑定给新目标。

日志：~/robot_nav_ws/mission_logs/perception（可用 log_directory 覆盖）。
每次候选事件保存 request_id.npz（RGB BGR数组+原编码深度）、request_id.json
（候选、时间、相机内参、采集时TF、实际发送的图片）与 request_id_response.json
（实际模型响应）。写盘采用有限队列；安全/控制不依赖这些磁盘操作。

官方接口依据：
https://docs.ultralytics.com/models/yolo-world/
https://help.aliyun.com/zh/model-studio/qwen3-8-flash
https://help.aliyun.com/zh/model-studio/vision

验证：在具备 numpy 的 Python 中运行工作区 tests/test_perception_core.py。
测试含空场景零事件、稳定触发、请求去重、代次作废、地图补偿关联、遮挡重关联、
拒绝冷却、严格JSON校验、深度单位、空洞、背景歧义、TF缺失。
