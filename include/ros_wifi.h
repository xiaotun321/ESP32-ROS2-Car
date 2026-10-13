#pragma once
#include <stdint.h>
struct RosCommand {
    double linear, angular;
    uint32_t receivedMs;
};
bool rosWifiBegin();
bool rosWifiConnected();
bool rosWifiTakeCommand(RosCommand &command);
