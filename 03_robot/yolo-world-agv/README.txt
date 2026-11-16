YOLO-World 垃圾桶识别：Jetson AGV 部署与复测

部署目录（小车）：/home/dase-orinnx/yolo-world-agv
本机备份目录：/Users/joe/HKU/DASE7143/yolo-world-agv

运行 60 帧摄像头测试（SSH 无需桌面）：
  ssh orinnx
  cd ~/yolo-world-agv
  ./run.sh --frames 60 --conf 0.15

在小车图形桌面的终端中显示检测窗口：
  cd ~/yolo-world-agv
  ./run.sh --frames 0 --show
按 q 或 Ctrl-C 退出；该命令会持续保存每帧原图与标注图，测试完成后及时退出。

提高置信度门槛：
  ./run.sh --frames 60 --conf 0.25

修改类别（首次已缓存 CLIP 权重，无需再下载）：
  ./run.sh --classes 'trash can' 'garbage bin' --frames 60

每次运行创建独立 results/YYYYMMDD_HHMMSS 目录，包含：
  raw/                 每帧原始图像
  annotated/           每帧检测框、提示词和置信度
  detections.jsonl     逐帧类别、置信度、像素坐标和推理耗时
  summary.json         帧数、检测出现频次、最大置信度、平均推理耗时

默认模型：YOLOv8s-Worldv2，未进行垃圾桶专项训练。
默认提示词：trash can, garbage bin, waste bin, dustbin, recycling bin。
模型将这些提示词作为垃圾桶的候选称呼；标签不能用于判断实际垃圾分类用途。
类别无关 NMS 合并同一物体的同义词重复框，IoU 阈值 0.5。
默认输入尺寸 640，FP16，CUDA device 0，采样间隔至少 0.25 秒。
检测频次和置信度均不等于准确率；需结合标注图判断框是否落在真正垃圾桶上。

摄像头：Intel RealSense D455f 的 RGB V4L2 接口，当前 /dev/video4。
不需要启动 ROS2 摄像头节点。该测试使用彩色图像，不测量距离。
重新插拔后编号可能变化，可通过 ls -l /dev/v4l/by-id/ 检查，
并使用 ./run.sh --source /dev/videoN 指定实际彩色接口。
若摄像头被其他进程占用，请先确认占用程序，不要同时直接打开同一接口。

环境：Ubuntu 22.04.5 / aarch64 / JetPack 6.2.1 / L4T 36.4.4 / CUDA 12.6。
Python 3.10 独立 .venv，Torch 2.8.0 和 Torchvision 0.23.0 使用 Jetson CUDA wheel，
Ultralytics 8.3.233，NumPy 1.26.4，OpenCV 4.11.0.86。
requirements-lock.txt 记录实际安装版本。重建使用 bash install.sh；
脚本先安装 Jetson 专用 Torch/Torchvision，再解析其余依赖。
模型权重与文本向量已保存在 models/，默认运行无需联网。

诊断：
  .venv/bin/pip check
  .venv/bin/python -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())'
  .venv/bin/python compare_prompts.py

compare_prompts.py 比较同一已保存正样本和初始场景中的五种单独提示词及组合提示词。
该比较属于当前场景诊断，不是独立标注测试集上的准确率评测。

官方参考：
https://docs.ultralytics.com/models/yolo-world/
https://docs.ultralytics.com/guides/nvidia-jetson/
https://github.com/openai/CLIP
https://pypi.jetson-ai-lab.io/jp6/cu126/
