// config.h
#pragma once
#include <stdint.h>
#include "driver/gpio.h"

namespace Config {

// TWAI transceiver pins (SN65HVD230), replaces CS/INT pins from MCP2515
static constexpr gpio_num_t kCanTxPin = GPIO_NUM_4;  // set to your actual wiring
static constexpr gpio_num_t kCanRxPin = GPIO_NUM_5;  // set to your actual wiring

static constexpr uint8_t kMotorId = 69;

static constexpr int32_t kDefaultErpm = 800;
static constexpr int32_t kMaxErpm = 3000;
static constexpr bool kAutoStart = false;

static constexpr uint32_t kCommandPeriodMs = 100;
static constexpr uint32_t kStatusPeriodMs = 500;
static constexpr uint32_t kMotionUpdatePeriodMs = 20;

static constexpr int32_t kAccelStep = 80;
static constexpr int32_t kStopStep = 120;
static constexpr int32_t kEmergencyStep = 300;

}  // namespace Config