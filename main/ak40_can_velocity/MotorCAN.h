#pragma once
#include <stdint.h>
#include "driver/twai.h"
#include "status.h"

// Low-level CAN transport (ESP32 native TWAI) and AK40 servo CAN protocol adapter.
class MotorCAN {
public:
    MotorCAN(gpio_num_t txPin, gpio_num_t rxPin, uint8_t motorId);

    bool begin();
    void poll();
    bool servoConnect();
    bool reconnect();
    bool servoSetRPM(int32_t erpm);
    bool servoSetDuty(float duty);

    const Status& getStatus() const;
    Status& getStatus();

private:
    uint32_t servoExtId(uint8_t functionId) const;
    bool sendExt(uint32_t id29, const uint8_t* data, uint8_t len);
    bool readCanFrame(uint32_t timeoutMs);
    bool isHandshakeReply(const twai_message_t& frame) const;
    bool processServoFrame();
    void sendHandshakeReply();
    void appendI32BE(uint8_t* buf, int32_t value) const;

    gpio_num_t txPin_;
    gpio_num_t rxPin_;
    uint8_t motorId_;
    twai_message_t rxFrame_;
    Status status_;
};