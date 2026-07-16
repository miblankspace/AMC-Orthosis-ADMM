#pragma once
#include <stdint.h>

class MotionController;

// Parses serial commands from USB/UART console and dispatches motion requests.
class SerialCommand {
public:
    explicit SerialCommand(MotionController& motion);

    // Starts a dedicated RX task (call once from app_main).
    void begin();

    // Applies any complete lines received by the RX task.
    void update();

    bool consumeStatusRequest();
    bool consumeReconnectRequest();
    bool consumeEmergencyStopRequest();

private:
    void handleLine(const char* line);
    static void trimInPlace(char* line);
    static char lower(char c);
    int32_t parseValue(const char* line) const;
    static void rxTask(void* arg);

    MotionController& motion_;
    bool statusRequested_;
    bool reconnectRequested_;
    bool emergencyStopRequested_;
    uint32_t lastHelpMs_ = 0;

    static constexpr size_t kLineBufSize = 64;
    void* lineQueue_ = nullptr;  // QueueHandle_t
};
