#include "status.h"
#include <stdio.h>
#include "MotionController.h"

Status::Status()
    : hasStatus_(false),
      positionDeg_(0.0f),
      measuredErpm_(0.0f),
      temperatureC_(0),
      error_(0),
      rawCurrent_(0) {}

void Status::updateFromServoStatusFrame(const uint8_t* data, uint8_t length)
{
    if (length < 8) {
        return;
    }

    const int16_t posRaw = (int16_t)((data[0] << 8) | data[1]);
    const int16_t spdRaw = (int16_t)((data[2] << 8) | data[3]);

    rawCurrent_ = (int16_t)((data[4] << 8) | data[5]);
    positionDeg_ = posRaw * 0.1f;
    measuredErpm_ = spdRaw * 10.0f;
    temperatureC_ = (int8_t)data[6];
    error_ = data[7];
    hasStatus_ = true;
}

bool Status::hasStatus() const { return hasStatus_; }
float Status::getPositionDeg() const { return positionDeg_; }
float Status::getMeasuredErpm() const { return measuredErpm_; }
int8_t Status::getTemperatureC() const { return temperatureC_; }
uint8_t Status::getError() const { return error_; }
int16_t Status::getRawCurrent() const { return rawCurrent_; }
float Status::getCurrentAmps() const { return rawCurrent_ * 0.01f; }

float Status::getEstimatedTorqueNm() const
{
    static constexpr float kTorquePerAmp = 5.0f / 60.0f;
    return getCurrentAmps() * kTorquePerAmp;
}

void Status::print(const MotionController& motion) const
{
    // printf (not ESP_LOGI) - kept plain so ak40_dashboard.py's key=value
    // regex parsing over serial isn't disturbed by log level/color codes.
    printf("state=%s  target_erpm=%ld  current_erpm=%ld  measured_erpm=%.0f  "
           "pos=%.1f  tempC=%d  err=%d  current=%.2fA  current_raw=%d  torque_est=%.2fNm\n",
           motion.getStateName(),
           (long)motion.getTargetERPM(),
           (long)motion.getCurrentERPM(),
           measuredErpm_,
           positionDeg_,
           temperatureC_,
           error_,
           getCurrentAmps(),
           rawCurrent_,
           getEstimatedTorqueNm());
}