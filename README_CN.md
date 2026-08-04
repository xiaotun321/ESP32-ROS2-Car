# ESP32-S3 + TB6612FN 开环底盘固件

## 功能

- 通过 WiFi UDP 连接树莓派 micro-ROS Agent；
- 订阅 `/cmd_vel`；
- 按差速模型控制左右两个无码盘直流电机；
- 200 ms 无命令立即停车；
- micro-ROS Agent 断开时拉低 STBY 并停车；
- PWM 斜坡限制，降低突然启动/反转的电流冲击。

该版本故意不发布 `/odom` 和 `/imu/data`，避免把无码盘电机的估算数据误当成真实传感器数据。

## 接线

| TB6612FN | ESP32-S3 |
|---|---:|
| PWMA | GPIO14 |
| AIN1 | GPIO15 |
| AIN2 | GPIO16 |
| PWMB | GPIO17 |
| BIN1 | GPIO18 |
| BIN2 | GPIO21 |
| STBY | GPIO47 |

ESP32、驱动板和电机电源必须共地。电机电源接驱动板 VM，不要从 ESP32 供电。

## 编译烧录

用 VS Code 打开 `esp32/tb6612_open_loop` 文件夹，然后执行 PlatformIO 的 Build 和 Upload。

当前版本按 WiFi 配置：

- WiFi 名称：`MIFI-38FF`
- WiFi 密码：在 `platformio.ini` 的 `MICRO_ROS_WIFI_PASSWORD` 中配置
- micro-ROS Agent UDP 端口：`8888`

ESP32 需要知道树莓派连接 `MIFI-38FF` 后的 IP。先在树莓派执行：

```bash
hostname -I
```

然后把 `platformio.ini` 里的这一行改成树莓派实际 IP：

```ini
-D MICRO_ROS_AGENT_IP=\"192.168.0.100\"
```

树莓派启动 Agent 时使用：

```bash
ros2 launch tb6612_car_ros2 car_bringup.launch.py transport:=udp4 udp_port:=8888
```

`platformio.ini` 当前按树莓派实际环境设置为 ROS 2 Jazzy。如果以后改用 Humble，将：

```ini
board_microros_distro = jazzy
```

改成：

```ini
board_microros_distro = humble
```

## 首次调试

先将轮子架空。若向前命令时某侧反转，修改 `main.cpp`：

```cpp
constexpr bool LEFT_MOTOR_INVERTED = true;
constexpr bool RIGHT_MOTOR_INVERTED = false;
```

不要同时交换接线和修改反转参数，否则容易混淆。

需要按实车调整：

- `WHEEL_DISTANCE_M`：左右轮中心距离；
- `MAX_WHEEL_SPEED_MPS`：最大速度映射值；
- `PWM_MIN_START`：能够可靠起步的最小 PWM；
- `PWM_SLEW_PER_TICK`：加减速快慢。
