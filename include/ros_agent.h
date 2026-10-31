#pragma once
#include <stdint.h>
struct RosCommand {
    double linear, angular;
    uint32_t receivedMs;
};
bool rosAgentBegin();
bool rosAgentConnected();
bool rosAgentTakeCommand(RosCommand &command);
