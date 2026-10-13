#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <atomic>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "ros_wifi.h"

#ifndef MICRO_ROS_WIFI_SSID
#error Define MICRO_ROS_WIFI_SSID and other Wi-Fi options in platformio.ini
#endif
#ifndef MICRO_ROS_DOMAIN_ID
#define MICRO_ROS_DOMAIN_ID 0
#endif

namespace {
QueueHandle_t commands = nullptr;
std::atomic<bool> connected{false};
rclc_support_t support{};
rcl_node_t node{};
rcl_subscription_t subscription{};
rclc_executor_t executor{};
rcl_allocator_t allocator{}; // Executor retains this pointer for its whole lifetime.
geometry_msgs__msg__Twist twist{};
bool supportReady = false, nodeReady = false, subscriptionReady = false, executorReady = false;
micro_ros_agent_locator locator;

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

void networkTask(void *) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(MICRO_ROS_WIFI_SSID, MICRO_ROS_WIFI_PASSWORD);
    uint32_t lastReconnect = millis(), lastPing = 0;
    bool wifiWasConnected = false;
    // Register UDP transport directly; upstream's convenience helper waits forever
    // for Wi-Fi. The network task must be able to retry without blocking control.
    locator.address.fromString(MICRO_ROS_AGENT_IP);
    locator.port = MICRO_ROS_AGENT_PORT;
    rmw_uros_set_custom_transport(false, &locator, arduino_wifi_transport_open,
        arduino_wifi_transport_close, arduino_wifi_transport_write, arduino_wifi_transport_read);
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            wifiWasConnected = false;
            if (supportReady) destroyEntities();
            connected.store(false);
            if (millis() - lastReconnect >= 10000) {
                WiFi.reconnect(); lastReconnect = millis();
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (!wifiWasConnected) {
            Serial.printf("WiFi connected: local=%s agent=%s:%u\n",
                WiFi.localIP().toString().c_str(), MICRO_ROS_AGENT_IP,
                static_cast<unsigned>(MICRO_ROS_AGENT_PORT));
            wifiWasConnected = true;
        }
        if (!connected.load()) {
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                if (createEntities()) {
                    Serial.printf("ROS connected: /tb6612_base_node -> %s:%u domain=%u\n",
                        MICRO_ROS_AGENT_IP, static_cast<unsigned>(MICRO_ROS_AGENT_PORT),
                        static_cast<unsigned>(MICRO_ROS_DOMAIN_ID));
                    lastPing = millis();
                } else destroyEntities();
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        const auto result = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        if (result != RCL_RET_OK && result != RCL_RET_TIMEOUT) {
            destroyEntities(); continue;
        }
        if (millis() - lastPing >= 200) {
            lastPing = millis();
            if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) {
                destroyEntities(); Serial.println("ROS disconnected; waiting for agent");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
}

bool rosWifiBegin() {
    commands = xQueueCreate(1, sizeof(RosCommand));
    if (!commands) return false;
    // Only this task touches RCL. Motor GPIO remains owned by Arduino loop.
    if (xTaskCreatePinnedToCore(networkTask, "micro_ros_wifi", 16384, nullptr, 1, nullptr, 0) != pdPASS) {
        vQueueDelete(commands); commands = nullptr; return false;
    }
    return true;
}
bool rosWifiConnected() { return connected.load(); }
bool rosWifiTakeCommand(RosCommand &command) {
    return commands && xQueueReceive(commands, &command, 0) == pdTRUE;
}
