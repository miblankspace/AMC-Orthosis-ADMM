#include "SerialCommand.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "MotionController.h"

static const char* TAG = "SerialCmd";

SerialCommand::SerialCommand(MotionController& motion)
    : motion_(motion),
      statusRequested_(false),
      reconnectRequested_(false),
      emergencyStopRequested_(false) {}

void SerialCommand::begin()
{
    // Prefer interrupt-driven USB Serial/JTAG RX (works with host "Send" boxes).
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = 512;
    cfg.tx_buffer_size = 512;
    const esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err == ESP_OK) {
        usb_serial_jtag_vfs_use_driver();
        ESP_LOGI(TAG, "USB Serial/JTAG driver installed for command RX");
    } else {
        // Console may already own the port; still try read_bytes / getchar.
        ESP_LOGW(TAG, "USB Serial/JTAG driver install: %s (using existing console)",
                 esp_err_to_name(err));
    }

    lineQueue_ = xQueueCreate(8, kLineBufSize);
    xTaskCreatePinnedToCore(
        &SerialCommand::rxTask,
        "serial_rx",
        4096,
        this,
        configMAX_PRIORITIES - 1,
        nullptr,
        0);
    printf("Serial RX ready — send: v 500 | s | e | i | h\n");
}

void SerialCommand::rxTask(void* arg)
{
    auto* self = static_cast<SerialCommand*>(arg);
    char line[kLineBufSize];
    size_t len = 0;
    int64_t lastByteUs = 0;

    vTaskDelay(pdMS_TO_TICKS(800));

    auto commit = [&]() {
        if (len == 0) {
            return;
        }
        line[len] = '\0';
        if (self->lineQueue_ != nullptr) {
            xQueueSend(static_cast<QueueHandle_t>(self->lineQueue_), line, 0);
        }
        len = 0;
    };

    while (true) {
        uint8_t byte = 0;
        int n = usb_serial_jtag_read_bytes(&byte, 1, 0);
        if (n <= 0) {
            // Fallback if driver path has nothing (or not installed).
            const int c = fgetc(stdin);
            if (c != EOF) {
                byte = static_cast<uint8_t>(c);
                n = 1;
                clearerr(stdin);
            }
        }

        if (n > 0) {
            lastByteUs = esp_timer_get_time();
            const char ch = static_cast<char>(byte);

            if (ch == '\n' || ch == '\r') {
                commit();
                continue;
            }
            if (ch < 32 || ch > 126) {
                continue;
            }
            if (len < kLineBufSize - 1) {
                line[len++] = ch;
            }

            // Single-letter commands don't need Enter.
            if (len == 1 &&
                (ch == 's' || ch == 'S' || ch == 'e' || ch == 'E' ||
                 ch == 'i' || ch == 'I' || ch == 'h' || ch == 'H')) {
                commit();
            }
            continue;
        }

        // If a partial "v 500" was typed without Enter, commit after short idle.
        if (len > 0 && (esp_timer_get_time() - lastByteUs) > 400000) {
            commit();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void SerialCommand::update()
{
    if (lineQueue_ == nullptr) {
        return;
    }

    char line[kLineBufSize];
    while (xQueueReceive(static_cast<QueueHandle_t>(lineQueue_), line, 0) == pdTRUE) {
        handleLine(line);
    }
}

bool SerialCommand::consumeStatusRequest()
{
    const bool requested = statusRequested_;
    statusRequested_ = false;
    return requested;
}

bool SerialCommand::consumeReconnectRequest()
{
    const bool requested = reconnectRequested_;
    reconnectRequested_ = false;
    return requested;
}

bool SerialCommand::consumeEmergencyStopRequest()
{
    const bool requested = emergencyStopRequested_;
    emergencyStopRequested_ = false;
    return requested;
}

char SerialCommand::lower(char c)
{
    return (char)tolower((unsigned char)c);
}

void SerialCommand::trimInPlace(char* line)
{
    char* start = line;
    while (*start == ' ' || *start == '\t') {
        ++start;
    }
    if (start != line) {
        memmove(line, start, strlen(start) + 1);
    }

    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\t')) {
        line[--n] = '\0';
    }
}

void SerialCommand::handleLine(const char* raw)
{
    char line[kLineBufSize];
    strncpy(line, raw, kLineBufSize - 1);
    line[kLineBufSize - 1] = '\0';
    trimInPlace(line);

    if (line[0] == '\0') {
        return;
    }

    printf("cmd: %s\n", line);

    if (strncmp(line, "Use:", 4) == 0 ||
        strncmp(line, "TWAI", 4) == 0 ||
        strncmp(line, "state=", 6) == 0 ||
        strncmp(line, "I (", 3) == 0 ||
        strncmp(line, "W (", 3) == 0 ||
        strncmp(line, "E (", 3) == 0) {
        return;
    }

    if (line[1] == '\0') {
        switch (lower(line[0])) {
            case 's':
                motion_.stop();
                printf("stop\n");
                return;
            case 'i':
                statusRequested_ = true;
                return;
            case 'h':
                reconnectRequested_ = true;
                printf("reconnect requested\n");
                return;
            case 'e':
                emergencyStopRequested_ = true;
                return;
            default:
                break;
        }
    }

    if (lower(line[0]) == 'v') {
        const int32_t erpm = parseValue(line);
        if (erpm > 0) {
            motion_.moveUp(erpm);
            printf("target +%ld erpm\n", (long)erpm);
            return;
        }
        if (erpm < 0) {
            motion_.moveDown(-erpm);
            printf("target %ld erpm\n", (long)erpm);
            return;
        }
        motion_.stop();
        printf("stop\n");
        return;
    }

    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - lastHelpMs_ > 2000) {
        printf("Use: v <erpm> | s | e | i | h\n");
        lastHelpMs_ = now;
    }
}

int32_t SerialCommand::parseValue(const char* line) const
{
    const char* p = line + 1;
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    return (int32_t)strtol(p, nullptr, 10);
}
