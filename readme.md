```bash
# 手动创建工程
~/.platformio/penv/bin/pio project init \
  --project-dir /home/shuaijie/ROS2_Car \
  --board esp32-s3-devkitc-1 \
  --project-option "framework=arduino" \
  --sample-code \
  --ide vscode

# 命令行中进行编译
cd /home/shuaijie/ROS2_Car/esp32-micro_ros
~/.platformio/penv/bin/pio run
```