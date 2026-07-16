#pragma once

#include <Arduino.h>

namespace Config {

static constexpr uint8_t kCanCsPin = 10;
static constexpr uint8_t kCanIntPin = 2;
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
