# DASE7143 · 代码与工程配置

私有仓库，整理日期 2026-11-16。按用户确认范围，仅保存代码、工程配置、测试及工程运行说明；课件、报告、图片、视频、验证数据和旧 Git 历史留在本地，不在此仓库中上传。不使用加密备份。

| 目录 | 内容 |
| --- | --- |
| `01_course_materials/` | M1-L2、M1-L3 实验源码 |
| `02_coursework/coursework1/code/` | Coursework 1 原代码包中提取的源码 |
| `02_coursework/coursework2/reference/` | 早期机器人源码审阅快照 |
| `03_robot/robot/` | ROS 2 底盘、雷达和 VLM 基础驱动 |
| `03_robot/robot_nav_ws/` | SLAM / Nav2 导航任务及控制台 |
| `03_robot/robot_option_b_ws/` | Option B 隐藏目标探索工程 |
| `03_robot/robot_rgbd_ws/` | RGB-D 搜索接近工程 |
| `03_robot/yolo-world-agv/` | 目标识别程序及依赖锁定 |
| `04_operations/` | 依赖配置、TF、建图和验证工具代码 |

工程内部路径保持不变。部署时按各工程 README 放至 `~/robot_*_ws` 等原有目录；`colcon build --symlink-install` 可重建构建产物。文档提及的图像、模型权重和实测记录仍在本地，并未随代码上传。

两份旧 `simple_vlm/config.yaml` 中的 API 密钥已清空。运行时设置 `DASHSCOPE_API_KEY`；原始凭据仅保留在本地配置。`archive-manifest.json` 提供全部归档源码的 SHA-256 校验值。已有测试和实机结果按原记录理解，本次仅整理及验证文件完整性。

恢复全部已上传代码：

```bash
git clone https://github.com/Joerange/DASE7143.git
```
