#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "Config.h"
#include "microphone.h"
#include "edge_impulse.h"
#include "MotionController.h"
#include "state_machine.h"
#include "MotorCAN.h"
#include "SerialCommand.h"

static const char* TAG = "Main";

static uint32_t millis()
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

extern "C" void app_main()
{
    init_i2s();
    start_mic_task();
    edge_impulse_init();

    MotorCAN motor(Config::kCanTxPin, Config::kCanRxPin, Config::kMotorId);
    MotionController motion(
        Config::kMaxErpm,
        Config::kAccelStep,
        Config::kStopStep,
        Config::kEmergencyStep,
        Config::kMotionUpdatePeriodMs);
    StateMachine stateMachine(motion);
    SerialCommand serialCommand(motion);
    serialCommand.begin();

    ESP_LOGI(TAG, "Starting AK40 voice + serial control");

    if (!motor.begin()) {
        ESP_LOGE(TAG, "TWAI CAN init failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (!motor.servoConnect()) {
        ESP_LOGW(TAG, "No motor yet - send h after power-on, or use serial v <erpm>");
    } else {
        ESP_LOGI(TAG, "Motor connected");
    }

    printf("Commands: v <erpm> | s=stop+hold | e=emergency/release | i=status | h=reconnect\n");
    printf("Voice: activate | up | down | stop | help (confidence >= 0.8)\n");
    printf("Max |ERPM|=%ld  AUTO_START=%s\n",
           (long)Config::kMaxErpm,
           Config::kAutoStart ? "on" : "off");

    if (Config::kAutoStart) {
        motion.moveUp(Config::kDefaultErpm);
    }

    uint32_t lastCommandMs = 0;
    uint32_t lastStatusPrintMs = 0;
    bool emergencyPassiveSent = false;

    while (true) {
        // Keep command path snappy: serial first, then CAN, then voice.
        serialCommand.update();

        if (serialCommand.consumeReconnectRequest()) {
            printf("%s\n", motor.reconnect() ? "Reconnected" : "Reconnect failed");
        }

        if (serialCommand.consumeEmergencyStopRequest()) {
            motion.emergencyStop();
            printf("Emergency stop: motor released passive\n");
        }

        motor.poll();

        ei_impulse_result_t result = edge_impulse_run();
        stateMachine.determineCommand(result);

        if (motor.getStatus().getError() != 0) {
            motion.emergencyStop();
        }

        motion.update();

        if (motion.isEmergencyStopActive()) {
            // e = release motor (passive / freewheel)
            if (!emergencyPassiveSent) {
                motor.servoSetDuty(0.0f);
                emergencyPassiveSent = true;
            }
        } else if (millis() - lastCommandMs >= Config::kCommandPeriodMs) {
            // s / idle at 0 ERPM = active hold (closed-loop velocity 0).
            // Keep streaming RPM, including 0, so the motor holds torque.
            motor.servoSetRPM(motion.getCurrentERPM());
            lastCommandMs = millis();
            emergencyPassiveSent = false;
        }

        const bool shouldPrintStatus =
            serialCommand.consumeStatusRequest() ||
            (motor.getStatus().hasStatus() &&
             millis() - lastStatusPrintMs >= Config::kStatusPeriodMs);

        if (shouldPrintStatus) {
            if (motor.getStatus().hasStatus()) {
                motor.getStatus().print(motion);
            } else {
                printf("No 0x29 status yet\n");
                motor.printBusStatus();
            }
            lastStatusPrintMs = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
