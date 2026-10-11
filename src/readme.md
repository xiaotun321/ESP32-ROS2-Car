# ESP32-S3 双轮 PWM 开环小车

硬件为 ESP32-S3、TB6612FNG、两台 MG310 电机和前方万向支撑轮。沿用原理图引脚；不使用 IMU、PID、自平衡或 ROS。左右轮直接接受 PWM 指令，编码器仅显示计数和实测转速，不调整输出，也不因编码器无信号而停止。

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

COM5 为示例，替换成实际端口。默认使用 UART0/USB 转串口；使用原生 USB 时在环境中添加 `build_flags = -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1` 并重新编译。上电保持停止。

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

已根据实车反馈“d 前进、w 右转、s 左转、a 后退”将右轮方向改为反向，配置为 `MOTOR_SIGN = {1, -1}`。这是电机安装/接线极性与软件正方向不一致导致的，不涉及 PID 或 IMU。重新上传后发送 `version`，应看到 `firmware=pwm-r2-right-inverted motor_sign=1,-1`；仅编译不会更新板上程序。若收到未知指令或不是该版本，检查上传端口和工程。若版本正确仍有该现象，分别发送 `pwm 80 0` 和 `pwm 0 80`，核对实际转动的是哪侧车轮以及方向，再校准左右通道与极性。

默认 PWM 上限 ±180（满量程 255），超限时左右同比例缩放。快捷键幅度可修改 `JOG_PWM`。已移除 `ticks`、`rpm`、`cmd_vel`、`zero` 指令及里程计；RPM 仅作为观测数据。PWM 数值不是转速，开环下负载和电池电压变化会使转速变化，两轮相同 PWM 也不保证走直线。

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
