#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>  // 提供 micro-ROS 通信层接口

#include <geometry_msgs/msg/twist.h>    // 标准速度消息类型 Twist

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
#include <WiFi.h>
#endif

// -------------------- TB6612FN 接线 --------------------
constexpr uint8_t PWMA_PIN = 14;
constexpr uint8_t AIN1_PIN = 15;
constexpr uint8_t AIN2_PIN = 16;
constexpr uint8_t PWMB_PIN = 17;
constexpr uint8_t BIN1_PIN = 18;
constexpr uint8_t BIN2_PIN = 21;
constexpr uint8_t STBY_PIN = 47;

// 如果某一侧正反方向与预期相反，只修改对应值。
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;

// -------------------- 车辆控制参数 --------------------
constexpr float WHEEL_DISTANCE_M = 0.224F;       // 左右轮中心距离，需按实车测量
constexpr float MAX_WHEEL_SPEED_MPS = 0.50F;     // 仅用于把 cmd_vel 映射为 PWM
constexpr int PWM_MAX = 255;
constexpr int PWM_MIN_START = 55;                // 克服静摩擦；若低速抖动可调整
constexpr int PWM_SLEW_PER_TICK = 18;            // 每 20 ms 最大 PWM 变化，减小启动冲击
constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t COMMAND_TIMEOUT_MS = 200;

constexpr uint32_t SERIAL_BAUDRATE = 921600;
constexpr char NODE_NAME[] = "tb6612_base_node";
constexpr char CMD_VEL_TOPIC[] = "cmd_vel";

#ifndef MICRO_ROS_WIFI_SSID
#define MICRO_ROS_WIFI_SSID "MIFI-38FF"
#endif

#ifndef MICRO_ROS_WIFI_PASSWORD
#define MICRO_ROS_WIFI_PASSWORD "jie123456"
#endif

#ifndef MICRO_ROS_AGENT_IP
// Replace this with the Raspberry Pi WiFi IP shown by: hostname -I
#define MICRO_ROS_AGENT_IP "192.168.0.100"
#endif

#ifndef MICRO_ROS_AGENT_PORT
#define MICRO_ROS_AGENT_PORT 8888
#endif

// Arduino-ESP32 2.x LEDC API。
constexpr uint8_t PWM_CHANNEL_A = 0;
constexpr uint8_t PWM_CHANNEL_B = 1;
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;

rcl_allocator_t allocator;              // ROS2 内存分配器。micro-ROS创建节点、订阅者等实体时需要它。
rclc_support_t support;                 // 保存ROS2 上下文、初始化状态等基础支持信息。
rcl_node_t node;                        // 节点对象
rcl_subscription_t cmd_vel_subscriber;  // /cmd_vel 订阅者对象
rclc_executor_t executor;               // 执行器，用于检测消息并调用订阅回调。类似Python中的 rclpy.spin_once(node)
geometry_msgs__msg__Twist cmd_vel_msg;  // 用于保存接收到的 Twist 消息。


// Agent连接状态机
enum class AgentState {
  WAITING,        // 1.等待Agent出现
  AVAILABLE,      // 2.已ping通Agent  准备创建ROS实体
  CONNECTED,      // 3.ROS实体创建成功，正常通信和控制
  DISCONNECTED    // 4.Agent失联，准备清理并重新连接
};


// 处于运行状态的变量
AgentState agent_state = AgentState::WAITING;
uint32_t last_command_ms = 0;           // 上一次收到 /cmd_vel 的时间，用于判断200ms命令超时。
uint32_t last_control_ms = 0;           // 上一次更新电机控制的时间，用于实现20ms控制周期。
uint32_t last_agent_check_ms = 0;       // 上一次检测Agent的时间。
bool command_received = false;          // 是否至少收到过一条速度命令。
int current_left_pwm = 0;
int current_right_pwm = 0;


// 限制PWM范围 -255~255
static int clampPwm(int value) {
  return constrain(value, -PWM_MAX, PWM_MAX);
}

// 应用最低启动PWM
static int applyMinimumPwm(int value) {
  if (value == 0) return 0;
  const int magnitude = abs(value);
  if (magnitude < PWM_MIN_START) {
    return value > 0 ? PWM_MIN_START : -PWM_MIN_START;
  }
  return value;
}

// PWM渐变
static int slewTowards(int current, int target) {
  if (target > current + PWM_SLEW_PER_TICK) return current + PWM_SLEW_PER_TICK;
  if (target < current - PWM_SLEW_PER_TICK) return current - PWM_SLEW_PER_TICK;
  return target;
}


// 控制单个电机
static void writeOneMotor(uint8_t in1, uint8_t in2, uint8_t pwm_channel,
                          int signed_pwm, bool inverted) {
  int output = clampPwm(signed_pwm);
  if (inverted) output = -output;

  if (output > 0) {  // 正转
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwm_channel, output);
  } else if (output < 0) {  // 反转
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwm_channel, -output);
  } else {
    // 滑行停止，避免在调试阶段频繁短路制动造成大电流。
    ledcWrite(pwm_channel, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// 同时更新左右电机
static void applyMotorOutputs() {
  writeOneMotor(AIN1_PIN, AIN2_PIN, PWM_CHANNEL_A,
                current_left_pwm, LEFT_MOTOR_INVERTED);
  writeOneMotor(BIN1_PIN, BIN2_PIN, PWM_CHANNEL_B,
                current_right_pwm, RIGHT_MOTOR_INVERTED);
}

// 强制停车
static void hardStop(bool disable_driver) {
  current_left_pwm = 0;
  current_right_pwm = 0;
  applyMotorOutputs();
  if (disable_driver) digitalWrite(STBY_PIN, LOW);
}

// 接收速度消息
static void cmdVelCallback(const void *message) {
  const auto *cmd = static_cast<const geometry_msgs__msg__Twist *>(message);  // 将无类型指针转换成 Twist 指针：
  // 只接受差速底盘需要的 linear.x 和 angular.z；linear.y 被忽略。
  cmd_vel_msg.linear.x = cmd->linear.x;
  cmd_vel_msg.angular.z = cmd->angular.z;
  last_command_ms = millis();  // 记录收到消息的时间
  command_received = true;     // 标记已经收到过有效命令
}

// 运动控制
static void updateOpenLoopControl() {

  // 判断命令是否超时
  const uint32_t now = millis();
  const bool timed_out = !command_received ||
                         static_cast<uint32_t>(now - last_command_ms) >= COMMAND_TIMEOUT_MS;

  int target_left_pwm = 0;
  int target_right_pwm = 0;

  if (!timed_out) {
    const float linear = static_cast<float>(cmd_vel_msg.linear.x);   //读取线速度、角速度
    const float angular = static_cast<float>(cmd_vel_msg.angular.z);


    /*差速地盘运动学
    左轮速度 = 车体线速度 - 角速度 x 轮距 / 2
    右轮速度 = 车体线速度 + 角速度 x 轮距 / 2
    */
    float left_speed = linear - angular * WHEEL_DISTANCE_M * 0.5F;
    float right_speed = linear + angular * WHEEL_DISTANCE_M * 0.5F;


    // 同比例限速 找出左右轮速度绝对值较大的一个
    const float largest = max(abs(left_speed), abs(right_speed));
    if (largest > MAX_WHEEL_SPEED_MPS) {
      const float scale = MAX_WHEEL_SPEED_MPS / largest;
      left_speed *= scale;
      right_speed *= scale;
    }

    // 轮速转换为PWM 公式 目标PWM = 轮速 ÷ 最大轮速 × 255
    target_left_pwm = applyMinimumPwm(
        static_cast<int>(left_speed / MAX_WHEEL_SPEED_MPS * PWM_MAX));
    target_right_pwm = applyMinimumPwm(
        static_cast<int>(right_speed / MAX_WHEEL_SPEED_MPS * PWM_MAX));
  }

  // 超时立即归零；正常变速使用斜坡限制。
  if (timed_out) {
    current_left_pwm = 0;
    current_right_pwm = 0;
  } else {
    current_left_pwm = slewTowards(current_left_pwm, target_left_pwm);
    current_right_pwm = slewTowards(current_right_pwm, target_right_pwm);
  }
  applyMotorOutputs();
}


// 创建 ROS 实体
static bool createRosEntities() {
  allocator = rcl_get_default_allocator();  // 获取默认内存分配器
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;    // 初始化micro-ROS支持环境
  if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) return false; // 创建节点
  if (rclc_subscription_init_default(       // 创建订阅者
          &cmd_vel_subscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          CMD_VEL_TOPIC) != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;  // 创建执行器 类似rclpy.spin_once(node)
  // 把订阅者加入执行器, 当 cmd_vel_subscriber 收到新数据: 1.将数据写入 cmd_vel_msg 2.调用 cmdVelCallback
  if (rclc_executor_add_subscription(                                                     
          &executor, &cmd_vel_subscriber, &cmd_vel_msg,
          &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

  // 启动电机驱动器
  command_received = false;
  digitalWrite(STBY_PIN, HIGH);
  return true;
}

// 断线清理
static void destroyRosEntities() {
  hardStop(true);
  rmw_context_t *context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(context, 0);
  (void)rclc_executor_fini(&executor);
  (void)rcl_subscription_fini(&cmd_vel_subscriber, &node);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);
}

// 程序上电后，setup()只执行一次。 进行硬件和通信初始化
void setup() {
  pinMode(AIN1_PIN, OUTPUT);  // 配置方向引脚
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION_BITS);  // 配置左电机PWM
  ledcAttachPin(PWMA_PIN, PWM_CHANNEL_A);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION_BITS);  // 配置右电机PWM
  ledcAttachPin(PWMB_PIN, PWM_CHANNEL_B);

  // 上电立即停车
  hardStop(true);
  Serial.begin(SERIAL_BAUDRATE);
  delay(1500);

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
  IPAddress agent_ip;
  agent_ip.fromString(MICRO_ROS_AGENT_IP);
  char wifi_ssid[] = MICRO_ROS_WIFI_SSID;
  char wifi_password[] = MICRO_ROS_WIFI_PASSWORD;
  // 配置WiFi和Agent地址：连接指定WIFI; 配置UDP Agent IP; 配置UDP Agent端口
  set_microros_wifi_transports(   
      wifi_ssid, wifi_password, agent_ip, MICRO_ROS_AGENT_PORT);
  delay(2000);
#else
  set_microros_serial_transports(Serial);
#endif
}

// 执行完 setup() 后，会不断重复执行 loop()。
void loop() {
  const uint32_t now = millis();

  switch (agent_state) {
    case AgentState::WAITING:
      if (static_cast<uint32_t>(now - last_agent_check_ms) >= 500) {
        last_agent_check_ms = now;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {  // ping Agent 单次等待超时100ms  尝试次数 1
          agent_state = AgentState::AVAILABLE;
        }
      }
      break;

    case AgentState::AVAILABLE:
      if (createRosEntities()) {    // Agent能ping通 创建ROS实体
        agent_state = AgentState::CONNECTED;
      } else {
        destroyRosEntities();
        agent_state = AgentState::WAITING;
      }
      break;

    case AgentState::CONNECTED:     // 正常运行状态 处理订阅消息
      /*
      让 micro-ROS 执行器检查并处理已经收到的消息 若收到新的 /cmd_vel, 则调用 cmdVelCallback()
      rclc_executor_spin_some() 是 micro-ROS 的事件处理函数, 创建ROS实体时, 已经把 /cmd_vel 订阅者注册到了执行器
      ESP32 中的 micro-ROS 不像完整的ROS2那样 自动启动独立线程处理消息。 需要不断的调用rclc_executor_spin_some  才能让执行器处理通信事件
      */
      (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));  
      if (static_cast<uint32_t>(now - last_control_ms) >= CONTROL_PERIOD_MS) {  // 每20ms控制一次电机
        last_control_ms = now;
        updateOpenLoopControl();
      }
      if (static_cast<uint32_t>(now - last_agent_check_ms) >= 200) {   // 每200ms检查一次Agent
        last_agent_check_ms = now;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          agent_state = AgentState::DISCONNECTED;
        }
      }
      break;

    case AgentState::DISCONNECTED:
      destroyRosEntities();
      agent_state = AgentState::WAITING;
      break;
  }
}
