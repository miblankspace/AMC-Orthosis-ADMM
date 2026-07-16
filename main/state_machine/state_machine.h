#pragma once
#include "edge-impulse-sdk/classifier/ei_classifier_types.h"
#include "esp_timer.h"

enum class SystemState {
    Inactive,
    Active
};

enum class Command {
    Activate,
    Up,
    Down,
    Stop,
    Help,
    None
};

class StateMachine {
public:
    StateMachine();
    ~StateMachine();

    void determineCommand(ei_impulse_result_t res);
    void update(Command command);

private:
    static constexpr uint64_t kInactivityTimeoutUs = 5'000'000;
    static constexpr uint64_t kStopTimeoutUs       = 3'000'000;
    static constexpr float    kConfidenceThreshold = 0.8f;

    SystemState state = SystemState::Inactive;
    esp_timer_handle_t timeoutTimer = nullptr;

    void armTimer(uint64_t timeoutUs);
    void disarmTimer();
    static void onTimeout(void* arg);
    void handleTimeout();
    bool stopped = false;

    static Command labelToCommand(const char* label);
    static const char* stateToStr(SystemState s);
    static const char* commandToStr(Command c);
};