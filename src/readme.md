# ESP32-S3 双轮 PWM 开环小车

硬件为 ESP32-S3、TB6612FNG、两台 MG310 电机和前方万向支撑轮。沿用原理图引脚；不使用 IMU、PID 或自平衡。支持树莓派 ROS 2 Jazzy 通过 Wi-Fi micro-ROS 下发运动指令，以及本地串口 PWM 调试。电机保持开环，编码器仅显示计数和实测转速，不调整输出，也不因编码器无信号而停止。

用户提供的参数：轮径 48 mm、减速比 1:20、额定电压 7.4V、额定转速 400±13% RPM、额定电流 ≤500mA、堵转电流 ≤2A。车体尺寸 147.5×144 mm。开环控制不需要轮距。

编码器每电机轴转 13 个周期，输出轴每转为 13×20=260 个周期。固件统计 AB 两相所有边沿，因此每输出轴转为 **1040 个计数**，不是 260。参数集中在 `include/robot_config.h`；实测转一圈可进一步核对计数。

## 接线

引脚逐项核对自 `SCH_Schematic1_2026-09-01.pdf`。默认把 U8/A 路安装在左轮、U9/B 路安装在右轮；原理图本身未规定左右。

| 信号 | 左轮 A 路 / U8 | 右轮 B 路 / U9 |
| --- | --- | --- |
| PWM | GPIO14 / PWMA | GPIO17 / PWMB |
| IN1 | GPIO15 / AIN1 | GPIO18 / BIN1 |
| IN2 | GPIO16 / AIN2 | GPIO21 / BIN2 |
| 编码器 A（接口第 3 脚） | GPIO6 | GPIO12 |
| 编码器 B（接口第 4 脚） | GPIO7 | GPIO13 |
| 电机接口第 1 / 6 脚 | AO1 / AO2 | BO1 / BO2 |
| 电机接口第 2 / 5 脚 | GND / 编码器 +5V | GND / 编码器 +5V |

共用 STBY 为 GPIO47，VM 原图为 12V，所有信号共地。电机实际额定电压须与 VM 相符。不要把电机电源接入 ESP32 GPIO。

**原图中有两项需要实物核对的电平问题：**

- TB6612 的 VCC 原图接 5V，但数据手册要求控制高电平至少为 `0.7 × VCC`，5V 时就是 3.5V，ESP32 的 3.3V 输出不保证满足。建议将 TB6612 **逻辑 VCC 改为 3.3V**（VM 电机供电保持独立），或者采用合适的电平转换。确认模块上的 VCC 没有与其他 5V 电源硬连后再改线。
- 编码器原图由 5V 供电，不能据此判断 AB 输出类型。若 AB 为 5V 推挽输出，须降为 3.3V 电平再接 GPIO；若为开集电极输出，则确认上拉接到 3.3V，并排除模块内已有的 5V 上拉。软件 `INPUT_PULLUP` 不能把 5V 信号转换成 3.3V。

依据：[Toshiba TB6612FNG 数据手册](https://toshiba.semicon-storage.com/info/datasheet_en_20141001.pdf?did=10660)、[Espressif ESP32-S3 数据手册](https://documentation.espressif.com/esp32_s3_datasheet_en.pdf)。


电机规格图片明确标注 **7.4V**，原理图的 VM 标注为 12V；实际供电应按该电机额定电压选择，不能因为沿用原理图就默认这台电机可用 12V。PWM 限幅不是电源稳压。编码器图片标注供电支持 3.3–5V，且自带上拉整形；若按原图用 5V 供电，仍应核对实际 AB 高电平。

## 编译和使用

保留原工程板型 `esp32-s3-devkitm-1`，平台锁定 espressif32 6.11.0。使用 PlatformIO 终端：

```powershell
pio run
pio run --target upload --upload-port COM5
pio device monitor --port COM5 --baud 115200
```

COM5 为示例，替换成实际端口。按用户要求设置 `ARDUINO_USB_CDC_ON_BOOT=0`，本地调试使用 UART0/USB 转串口；Wi-Fi 与该串口独立。上电保持停止。

串口 115200，每行一条指令，以 LF 或 CRLF 结束：

| 指令 | 动作 |
| --- | --- |
| `w` | 前进：左右 PWM 均为 80 |
| `s` | 后退：左右 PWM 均为 -80 |
| `a` | 原地左转：左 -80、右 80 |
| `d` | 原地右转：左 80、右 -80 |
| `x` 或 `stop` | 关闭 STBY，滑行停止 |
| `pwm 80 0` | 左轮 PWM 80、右轮停止 |
| `pwm 0 80` | 左轮停止、右轮 PWM 80 |
| `pwm 80 100` | 独立设置两轮 PWM |
| `status` | 显示模式、故障、累计计数、计数/秒、目标 PWM、实际 PWM、RPM |
| `stream 1` / `stream 0` | 开启 / 关闭每秒 5 次状态输出 |
| `clear` | 停止并清除控制超时故障，不自动启动 |
| `help` | 指令帮助 |
| `version` | 显示固件版本和实际生效的电机方向配置 |

保留实车右轮反向修正 `MOTOR_SIGN = {1, -1}`。重新上传后发送 `version`，应看到 `firmware=pwm-r3-wifi-jazzy motor_sign=1,-1`；仅编译不会更新板上程序。若方向仍不对，分别发送 `pwm 80 0` 和 `pwm 0 80`，核对实际转动的是哪侧车轮以及方向。

默认 PWM 上限 ±180（满量程 255），超限时左右同比例缩放。快捷键幅度可修改 `JOG_PWM`。串口不提供 `ticks`、`rpm`、`cmd_vel`、`zero` 指令；ROS `/cmd_vel` 话题另见下文。RPM 仅作为串口观测数据，没有发布里程计。PWM 数值不是转速，开环下负载和电池电压变化会使转速变化，两轮相同 PWM 也不保证走直线。

运动指令需每 100 ms 刷新，500 ms 未刷新自动停车；手动发送一次最多运行约 0.5 秒。`status` 不刷新运动超时。正反切换插入 20 ms 零输出周期。控制循环超过 100 ms 会停机并锁定故障，使用 `clear` 清除。没有电流或堵转保护检测。

## 定时运动脚本

关闭串口监视器后执行，脚本自动每 100 ms 重发，时间到或 Ctrl+C 后发送停止：

```powershell
python -m pip install pyserial
python tools/drive.py --port COM5 --command "pwm 80 0" --seconds 0.3
python tools/drive.py --port COM5 --command "pwm 0 80" --seconds 0.3
python tools/drive.py --port COM5 --command "w" --seconds 2
python tools/drive.py --port COM5 --command "a" --seconds 1
```

默认指令为 `pwm 80 80`，默认持续 0.3 秒，最大 30 秒。初次架空车轮分别试转：正 PWM 应推动对应车轮向车体前进方向旋转；若相反，修改配置中对应 `MOTOR_SIGN` 为 -1。若编码器显示符号相反，独立修改 `ENCODER_SIGN`，它不会改变电机转向。若低 PWM 无法起转，可逐步增加 `pwm` 数值。

## 验证

`test/host/test_motion.cpp` 用 GPIO、串口、时钟桩运行实际固件代码，验证 PWM 不受编码器读数影响、四个方向、单轮控制、限幅、反转间隔、500 ms 失联停车、控制超时故障、非法指令、计数与时钟回绕及 RPM 换算。

在 Visual Studio x64 Native Tools 命令提示符下执行：

```bat
cl /nologo /EHsc /std:c++14 /Itest\host /Iinclude test\host\test_motion.cpp /Fo.pio\test_motion.obj /Fe.pio\test_motion.exe
.pio\test_motion.exe
```

固件为 `.pio/build/esp32-s3-devkitm-1/firmware.bin`。编译和主机测试不能代替烧录后的实际转向、接线与供电验证。

## 树莓派 Jazzy / Wi-Fi

`platformio.ini` 已设置 Wi-Fi 名称 804、用户提供的密码、Agent `192.168.0.107:8888`、`MICRO_ROS_DOMAIN_ID=0`。密码仅在配置中，不在运行日志中打印；分享工程前自行去除 Wi-Fi 凭据。树莓派连上同一局域网后用 `hostname -I` 确认地址，地址变化需修改宏并重新上传。

依赖固定为官方 `micro_ros_arduino v2.0.8-jazzy`。Windows 下使用 `tools/link_microros.py` 显式链接发布包中的 `src/esp32/libmicroros.a`：该版本的官方生成脚本对 ESP32 和 ESP32-S3 使用相同的工具链配置和 colcon 配置。适配范围为本工程固定的 Arduino 2.0.17 / espressif32 6.11.0；升级平台后需重新验证。官方另提供面向 PlatformIO 的源码构建方案，见 [micro_ros_platformio](https://github.com/micro-ROS/micro_ros_platformio)。当前选择预编译包以支持这台 Windows 开发机；首次编译需要联网下载依赖。

树莓派包 `D:/car/Raspberry-Pi-5_ROS2-Car/src/tb6612_car_ros2` 已修改键盘入口和 10 Hz 指令刷新逻辑。将更新后的包同步到树莓派的 ROS 工作区，在工作区根目录执行（假设该目录是 `~/car_ws`）：

```bash
cd ~/car_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tb6612_car_ros2 --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch tb6612_car_ros2 car_bringup.launch.py transport:=udp4 udp_port:=8888
```

第二个终端：

```bash
cd ~/car_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 run tb6612_car_ros2 keyboard_teleop
```

树莓派需已安装/构建 Jazzy 的 `micro_ros_agent`；可单独运行 `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888` 代替 launch（不要同时运行两个 Agent 占用同一个端口）。本固件只有 Wi-Fi micro-ROS 传输，不能使用上位机 launch 的 `transport:=serial`。

ROS 键盘布局：**w 前进、x 后退、a 左转、d 右转、s/空格停止**，与本地串口的 s/x 含义不同。持续按方向键移动；最后一次方向按键后默认 0.6 秒清零，可用 `--ros-args -p key_timeout:=0.8` 调整。Ctrl-C 退出时发送零指令。

ESP32 节点名 `/tb6612_base_node`，订阅 `/cmd_vel`、类型 `geometry_msgs/msg/Twist`、Best Effort/Volatile/Keep Last 1（兼容上位机默认 Reliable 发布者）。只使用 `linear.x` 和 `angular.z`：

```text
left_pwm  = linear.x * LINEAR_PWM_GAIN - angular.z * ANGULAR_PWM_GAIN
right_pwm = linear.x * LINEAR_PWM_GAIN + angular.z * ANGULAR_PWM_GAIN
```

默认 `linear.x=0.15` 对应 PWM 80/80，`angular.z=0.8` 对应 PWM -80/80。增益在 `robot_config.h` 中修改；这是开环映射，不是经过校准的车速或转速控制。没有 PID，没有 IMU，也不需要轮距。编码器仍只在串口显示。

网络连接和 ROS executor 在独立 FreeRTOS 任务中执行；控制循环不等待 Wi-Fi 或 Agent。失去 Agent/Wi-Fi 时停止 ROS 驱动的运动，断线自动重连，重连不会重放旧指令。500 ms 收不到新指令仍会停车。串口和 ROS 都能控制，最新运动指令生效；串口试车前退出 ROS 遥控节点，避免周期性 ROS 指令覆盖手动操作。串口 `stop` 不是锁定急停。

检查连接：

```bash
ros2 node list
ros2 topic info /cmd_vel -v
```

串口应出现 `WiFi connected: ...` 和 `ROS connected: /tb6612_base_node ...`；`status` 显示 `ros=connected` 及当前控制来源。如果 Wi-Fi 已连接但 ROS 未连接，检查树莓派 IP、UDP 8888、防火墙/AP 客户端隔离以及 Agent 是否已启动。节点存在但收不到命令时检查两端 `ROS_DOMAIN_ID` 是否一致。

新增主机测试覆盖 Twist 映射、断线停车、丢弃过期命令、重连不自动恢复、非法浮点数以及键盘心跳/松键超时。运行 `python -m unittest discover -s test -p test_pi_teleop.py -v` 检查实际树莓派 Python 源码（用 ROS/终端桩，无需本机安装 ROS）。尚需在实际 Wi-Fi 网络上完成两端联调。
