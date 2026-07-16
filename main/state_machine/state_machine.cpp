#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "state_machine.h"

static const char* TAG = "StateMachine";

StateMachine::StateMachine()
{
    esp_timer_create_args_t args = {
        .callback = &StateMachine::onTimeout,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "smTimeout",
        .skip_unhandled_events = true
    };
    esp_err_t err = esp_timer_create(&args, &timeoutTimer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
    }
}

StateMachine::~StateMachine()
{
    if (timeoutTimer) {
        esp_timer_stop(timeoutTimer);
        esp_timer_delete(timeoutTimer);
    }
}

void StateMachine::armTimer(uint64_t timeoutUs)
{
    esp_timer_stop(timeoutTimer);
    esp_err_t err = esp_timer_start_once(timeoutTimer, timeoutUs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to arm timer for %llu us: %s", timeoutUs, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Timer armed for %llu ms", timeoutUs / 1000);
}

void StateMachine::disarmTimer()
{
    esp_timer_stop(timeoutTimer);
}

void StateMachine::onTimeout(void* arg)
{
    static_cast<StateMachine*>(arg)->handleTimeout();
}

void StateMachine::handleTimeout()
{
    ESP_LOGW(TAG, "Timeout fired while state=%s stopped=%d -> transitioning to Inactive",
             stateToStr(state), stopped);
    state = SystemState::Inactive;
    stopped = false;
}

Command StateMachine::labelToCommand(const char* label)
{
    if (strcmp(label, "activate") == 0) return Command::Activate;
    if (strcmp(label, "up") == 0)       return Command::Up;
    if (strcmp(label, "down") == 0)     return Command::Down;
    if (strcmp(label, "stop") == 0)     return Command::Stop;
    if (strcmp(label, "help") == 0)     return Command::Help;
    return Command::None;     // "noise", "unknown"
}

void StateMachine::determineCommand(ei_impulse_result_t res)
{
    int bestIndex = -1;
    float bestValue = 0.0f;

    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
    {
        float value = res.classification[i].value;
        ESP_LOGI(TAG, "Label '%s': %.3f", res.classification[i].label, value);

        if (value > bestValue)
        {
            bestValue = value;
            bestIndex = static_cast<int>(i);
        }
    }

    if (bestIndex < 0 || bestValue < kConfidenceThreshold)
    {
        ESP_LOGI(TAG, "No confident classification (best=%.3f, threshold=%.3f) - ignoring",
                 bestValue, kConfidenceThreshold);
        return;
    }

    Command cmd = labelToCommand(res.classification[bestIndex].label);
    if (cmd == Command::None)
    {
        ESP_LOGI(TAG, "Top label '%s' is not a recognized command - ignoring",
                 res.classification[bestIndex].label);
        return;
    }

    ESP_LOGI(TAG, "Classified command: %s (confidence %.3f)",
             commandToStr(cmd), bestValue);
    update(cmd);
}

void StateMachine::update(Command command)
{
    ESP_LOGI(TAG, "update() called: command=%s, current state=%s",
             commandToStr(command), stateToStr(state));

    // Global emergency/help command
    if (command == Command::Help)
    {
        ESP_LOGW(TAG, "Help received - forcing Inactive");
        // motor.release();
        disarmTimer();
        state = SystemState::Inactive;
        stopped = false;
        return;
    }

    if (state == SystemState::Active)
    {
        switch (command)
        {
            case Command::Up:
                ESP_LOGI(TAG, "Motor: move up");
                //motor.moveup()
                stopped = false;
                armTimer(kInactivityTimeoutUs);
                break;

            case Command::Down:
                ESP_LOGI(TAG, "Motor: move down");
                //motor.movedown()
                stopped = false;
                armTimer(kInactivityTimeoutUs);
                break;

            case Command::Stop:
                ESP_LOGI(TAG, "Motor: stop");
                //motor.stop()
                stopped = true;
                armTimer(kStopTimeoutUs);
                break;

            default:
                break;
        }
    }
    else
    {
        if (command == Command::Activate)
        {
            ESP_LOGI(TAG, "Activating");
            state = SystemState::Active;
            stopped = false;
            armTimer(kInactivityTimeoutUs);
        }
        else
        {
            ESP_LOGI(TAG, "Ignored %s while Inactive", commandToStr(command));
        }
    }
}

// for logging
const char* StateMachine::stateToStr(SystemState s)
{
    switch (s) {
        case SystemState::Inactive: return "Inactive";
        case SystemState::Active:   return "Active";
    }
    return "Unknown";
}

// for logging
const char* StateMachine::commandToStr(Command c)
{
    switch (c) {
        case Command::Activate: return "Activate";
        case Command::Up:       return "Up";
        case Command::Down:     return "Down";
        case Command::Stop:     return "Stop";
        case Command::Help:     return "Help";
        case Command::None:     return "None";
    }
    return "Unknown";
}