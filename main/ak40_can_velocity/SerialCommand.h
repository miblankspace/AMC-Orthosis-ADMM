#pragma once
#include <stdint.h>

class MotionController;

// Parses serial commands (via ESP-IDF UART) and dispatches motion-intent requests.
class SerialCommand {
public:
    explicit SerialCommand(MotionController& motion);

    void update(); // call periodically; non-blocking

    bool consumeStatusRequest();
    bool consumeReconnectRequest();
    bool consumeEmergencyStopRequest();

private:
    void handleLine(const char* line);
    int32_t parseValue(const char* line) const;

    MotionController& motion_;
    bool statusRequested_;
    bool reconnectRequested_;
    bool emergencyStopRequested_;

    static constexpr size_t kLineBufSize = 64;
    char lineBuf_[kLineBufSize];
    size_t lineLen_ = 0;
};