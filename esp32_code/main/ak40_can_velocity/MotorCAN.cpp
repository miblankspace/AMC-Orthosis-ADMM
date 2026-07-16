#include "MotorCAN.h"

#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MotorCAN";

static uint32_t millis()
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

MotorCAN::MotorCAN(gpio_num_t txPin, gpio_num_t rxPin, uint8_t motorId)
    : txPin_(txPin), rxPin_(rxPin), motorId_(motorId) {}

bool MotorCAN::begin()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(txPin_, rxPin_, TWAI_MODE_NORMAL);
    // Larger TX queue helps when the bus is briefly busy.
    g_config.tx_queue_len = 10;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  // AK40 servo CAN = 1 Mbps
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_LOGI(TAG, "TWAI TX=GPIO%d RX=GPIO%d motorId=%u @1Mbps",
             (int)txPin_, (int)rxPin_, (unsigned)motorId_);

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed");
        return false;
    }
    return true;
}

void MotorCAN::poll()
{
    recoverIfBusOff();
    while (readCanFrame(0)) {
        processServoFrame();
    }
}

void MotorCAN::printBusStatus() const
{
    twai_status_info_t info = {};
    if (twai_get_status_info(&info) != ESP_OK) {
        printf("TWAI: status read failed\n");
        return;
    }

    const char* state = "UNKNOWN";
    switch (info.state) {
        case TWAI_STATE_STOPPED: state = "STOPPED"; break;
        case TWAI_STATE_RUNNING: state = "RUNNING"; break;
        case TWAI_STATE_BUS_OFF: state = "BUS_OFF"; break;
        case TWAI_STATE_RECOVERING: state = "RECOVERING"; break;
    }

    printf("TWAI state=%s tx_err=%lu rx_err=%lu tx_failed=%lu rx_missed=%lu "
           "arb_lost=%lu bus_err=%lu\n",
           state,
           (unsigned long)info.tx_error_counter,
           (unsigned long)info.rx_error_counter,
           (unsigned long)info.tx_failed_count,
           (unsigned long)info.rx_missed_count,
           (unsigned long)info.arb_lost_count,
           (unsigned long)info.bus_error_count);
}

void MotorCAN::recoverIfBusOff()
{
    twai_status_info_t info = {};
    if (twai_get_status_info(&info) != ESP_OK) {
        return;
    }

    // After bus-off recovery, TWAI ends in STOPPED and must be started again.
    if (info.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGW(TAG, "TWAI bus-off - initiating recovery");
        twai_initiate_recovery();
        return;
    }

    if (info.state == TWAI_STATE_STOPPED) {
        const esp_err_t err = twai_start();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "TWAI restarted after STOPPED");
        } else {
            ESP_LOGW(TAG, "TWAI restart failed: %s", esp_err_to_name(err));
        }
    }
}

bool MotorCAN::servoConnect()
{
    uint32_t end = millis() + 800;
    while (millis() < end) {
        if (readCanFrame(25) && processServoFrame()) {
            return true;
        }
    }

    const uint8_t zeros[8] = {0};
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        sendExt(servoExtId(0x2C), zeros, 8);
        end = millis() + 1500;
        while (millis() < end) {
            if (!readCanFrame(50)) {
                continue;
            }
            if (isHandshakeReply(rxFrame_) || processServoFrame()) {
                return true;
            }
        }
    }

    printBusStatus();
    return status_.hasStatus();
}

bool MotorCAN::reconnect() { return servoConnect(); }

bool MotorCAN::servoSetRPM(int32_t erpm)
{
    uint8_t payload[4];
    appendI32BE(payload, erpm);
    return sendExt(servoExtId(3), payload, 4);
}

bool MotorCAN::servoSetDuty(float duty)
{
    uint8_t payload[4];
    appendI32BE(payload, (int32_t)(duty * 100000.0f));
    return sendExt(servoExtId(0), payload, 4);
}

const Status& MotorCAN::getStatus() const { return status_; }
Status& MotorCAN::getStatus() { return status_; }

uint32_t MotorCAN::servoExtId(uint8_t functionId) const
{
    return ((uint32_t)functionId << 8) | motorId_;
}

bool MotorCAN::sendExt(uint32_t id29, const uint8_t* data, uint8_t len)
{
    recoverIfBusOff();

    twai_message_t tx = {};
    tx.identifier = id29 & 0x1FFFFFFF;
    tx.extd = 1;
    tx.data_length_code = len;
    for (uint8_t i = 0; i < len; ++i) {
        tx.data[i] = data[i];
    }

    const esp_err_t err = twai_transmit(&tx, pdMS_TO_TICKS(50));
    if (err == ESP_OK) {
        return true;
    }

    const uint32_t now = millis();
    if (now - lastTxFailLogMs_ >= 2000) {
        ESP_LOGW(TAG, "TX failed (%s)", esp_err_to_name(err));
        printBusStatus();
        lastTxFailLogMs_ = now;
    }
    return false;
}

bool MotorCAN::readCanFrame(uint32_t timeoutMs)
{
    return twai_receive(&rxFrame_, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

bool MotorCAN::isHandshakeReply(const twai_message_t& frame) const
{
    return frame.data_length_code >= 4 && frame.data[0] == 0xFA && frame.data[1] == 0xFB &&
           frame.data[2] == 0xFC && frame.data[3] == 0xFD;
}

bool MotorCAN::processServoFrame()
{
    if (!rxFrame_.extd) {
        return isHandshakeReply(rxFrame_);
    }

    const uint32_t id = rxFrame_.identifier & 0x1FFFFFFF;
    const uint8_t func = (uint8_t)(id >> 8);
    const uint8_t node = (uint8_t)(id & 0xFF);
    if (node != motorId_) {
        return false;
    }

    if (func == 0x2C) {
        sendHandshakeReply();
        return true;
    }

    if (func == 0x29 && rxFrame_.data_length_code == 8) {
        status_.updateFromServoStatusFrame(rxFrame_.data, rxFrame_.data_length_code);
        return true;
    }

    return isHandshakeReply(rxFrame_);
}

void MotorCAN::sendHandshakeReply()
{
    const uint8_t reply[8] = {0xFA, 0xFB, 0xFC, 0xFD, 0, 0, 0, 0};
    sendExt(servoExtId(0x2C), reply, 8);
}

void MotorCAN::appendI32BE(uint8_t* buf, int32_t value) const
{
    buf[0] = (uint8_t)(value >> 24);
    buf[1] = (uint8_t)(value >> 16);
    buf[2] = (uint8_t)(value >> 8);
    buf[3] = (uint8_t)value;
}
