#include <Arduino.h>
#include <atomic>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "ros_agent.h"

// The native USB port (HWCDC) carries the micro-ROS serial agent traffic.
// The console (command protocol and logs) stays on UART0 / Serial0.
#ifndef MICRO_ROS_SERIAL_BAUD
#define MICRO_ROS_SERIAL_BAUD 115200
#endif
#ifndef MICRO_ROS_DOMAIN_ID
#define MICRO_ROS_DOMAIN_ID 0
#endif

namespace {
constexpr uint32_t PING_INTERVAL_MS = 2000;
constexpr int PING_TIMEOUT_MS = 500;
constexpr uint8_t PING_ATTEMPTS = 3;
constexpr uint32_t WRITE_TIMEOUT_MS = 1000;
// The custom transport MTU is 512 B; the stock 256 B ring buffers are too small.
constexpr size_t CDC_BUFFER_SIZE = 2048;
constexpr uint32_t CDC_TX_TIMEOUT_MS = 500;

QueueHandle_t commands = nullptr;
std::atomic<bool> connected{false};
rclc_support_t support{};
rcl_node_t node{};
rcl_subscription_t subscription{};
rclc_executor_t executor{};
rcl_allocator_t allocator{}; // Executor retains this pointer for its whole lifetime.
geometry_msgs__msg__Twist twist{};
bool supportReady = false, nodeReady = false, subscriptionReady = false, executorReady = false;

// HWCDC::end() de-initialises the USB peripheral, so the host loses the CDC
// device entirely and the agent session dies with it. Open and close stay
// inert: the CDC is brought up once in rosAgentBegin().
bool agentTransportOpen(struct uxrCustomTransport *transport) {
    (void)transport;
    return true;
}
bool agentTransportClose(struct uxrCustomTransport *transport) {
    (void)transport;
    return true;
}
size_t agentTransportWrite(struct uxrCustomTransport *transport, const uint8_t *buf,
                           size_t len, uint8_t *errcode) {
    (void)transport;
    (void)errcode;
    size_t sent = 0;
    const uint32_t deadline = millis() + WRITE_TIMEOUT_MS;
    // A framed XRCE message must leave in one piece; retry partial writes.
    while (sent < len) {
        const size_t chunk = Serial.write(buf + sent, len - sent);
        if (chunk == 0) {
            if ((int32_t)(millis() - deadline) >= 0) break;
            delay(1);
            continue;
        }
        sent += chunk;
    }
    return sent;
}
size_t agentTransportRead(struct uxrCustomTransport * transport, uint8_t *buf,
                          size_t len, int timeout, uint8_t *errcode) {
    (void)transport;
    (void)errcode;
    if (len == 0) return 0;
    // Fill the buffer up to len bytes. The wait budget restarts whenever
    // bytes arrive, so a fragmented frame is not cut short mid-message.
    uint32_t deadline = millis() + (timeout > 0 ? (uint32_t)timeout : 0);
    size_t got = 0;
    while (got < len) {
        const int queued = Serial.available();
        if (queued > 0) {
            const size_t want = len - got;
            const size_t chunk = Serial.read(buf + got, (size_t)queued < want ? (size_t)queued : want);
            if (chunk == 0 || chunk == (size_t)-1) break;
            got += chunk;
            deadline = millis() + (timeout > 0 ? (uint32_t)timeout : 0);
            continue;
        }
        if ((int32_t)(millis() - deadline) >= 0) break;
        delay(1);
    }
    return got;
}

void cleanupResult(rcl_ret_t result) {
    if (result != RCL_RET_OK) rcl_reset_error();
}

void onTwist(const void *data) {
    const auto *msg = static_cast<const geometry_msgs__msg__Twist *>(data);
    const RosCommand command{msg->linear.x, msg->angular.z, millis()};
    xQueueOverwrite(commands, &command);
}

void destroyEntities() {
    connected.store(false);
    xQueueReset(commands);
    if (supportReady) {
        rmw_uros_set_context_entity_destroy_session_timeout(
            rcl_context_get_rmw_context(&support.context), 0);
    }
    if (executorReady) { cleanupResult(rclc_executor_fini(&executor)); executorReady = false; }
    if (subscriptionReady) { cleanupResult(rcl_subscription_fini(&subscription, &node)); subscriptionReady = false; }
    if (nodeReady) { cleanupResult(rcl_node_fini(&node)); nodeReady = false; }
    if (supportReady) { cleanupResult(rclc_support_fini(&support)); supportReady = false; }
    rcl_reset_error();
}

bool createEntities() {
    allocator = rcl_get_default_allocator();
    auto options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&options, allocator) != RCL_RET_OK) return false;
    if (rcl_init_options_set_domain_id(&options, MICRO_ROS_DOMAIN_ID) != RCL_RET_OK) {
        cleanupResult(rcl_init_options_fini(&options)); return false;
    }
    support = {};
    const auto result = rclc_support_init_with_options(&support, 0, nullptr, &options, &allocator);
    cleanupResult(rcl_init_options_fini(&options));
    if (result != RCL_RET_OK) return false;
    supportReady = true;
    node = rcl_get_zero_initialized_node();
    if (rclc_node_init_default(&node, "tb6612_base_node", "", &support) != RCL_RET_OK) return false;
    nodeReady = true;
    subscription = rcl_get_zero_initialized_subscription();
    auto qos = rmw_qos_profile_default;
    qos.depth = 1; // No replay of a backlog of velocity commands.
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    if (rclc_subscription_init(&subscription, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel", &qos) != RCL_RET_OK)
        return false;
    subscriptionReady = true;
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    executorReady = true;
    if (rclc_executor_add_subscription(&executor, &subscription, &twist, &onTwist, ON_NEW_DATA) != RCL_RET_OK)
        return false;
    xQueueReset(commands);
    connected.store(true);
    return true;
}

void agentTask(void *) {
    uint32_t lastPing = millis();
    for (;;) {
        if (!connected.load()) {
            // The agent may not be running yet; the ping also creates the
            // session-less handshake the agent expects.
            if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) == RMW_RET_OK) {
                if (createEntities()) {
                    Serial0.println("ROS connected: /tb6612_base_node (USB serial agent)");
                    lastPing = millis();
                } else {
                    destroyEntities();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        const auto result = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        if (result != RCL_RET_OK && result != RCL_RET_TIMEOUT) {
            destroyEntities();
            Serial0.println("ROS session error; waiting for agent");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (millis() - lastPing >= PING_INTERVAL_MS) {
            lastPing = millis();
            if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) != RMW_RET_OK) {
                destroyEntities();
                Serial0.println("ROS disconnected; waiting for agent");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
}

bool rosAgentBegin() {
    commands = xQueueCreate(1, sizeof(RosCommand));
    if (!commands) return false;
    // Native USB CDC, dedicated to the agent. Sized for the 512 B transport MTU.
    Serial.setRxBufferSize(CDC_BUFFER_SIZE);
    Serial.setTxBufferSize(CDC_BUFFER_SIZE);
    Serial.begin(MICRO_ROS_SERIAL_BAUD);
    Serial.setTxTimeoutMs(CDC_TX_TIMEOUT_MS);
    // framing = true: a byte stream needs XRCE framing, unlike the UDP packet mode.
    rmw_uros_set_custom_transport(true, nullptr, agentTransportOpen, agentTransportClose,
                                   agentTransportWrite, agentTransportRead);
    // Only this task touches RCL. Motor GPIO remains owned by the Arduino loop.
    if (xTaskCreatePinnedToCore(agentTask, "micro_ros_serial", 16384, nullptr, 1, nullptr, 0) != pdPASS) {
        vQueueDelete(commands); commands = nullptr; return false;
    }
    return true;
}
bool rosAgentConnected() { return connected.load(); }
bool rosAgentTakeCommand(RosCommand &command) {
    return commands && xQueueReceive(commands, &command, 0) == pdTRUE;
}
