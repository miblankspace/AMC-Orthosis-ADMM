#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "microphone.h"
#include "edge_impulse.h"
#include "state_machine.h"

static const char* TAG = "Main";

extern "C" void app_main()
{
    init_i2s();
    start_mic_task();
    edge_impulse_init();

    StateMachine stateMachine;

    ESP_LOGI(TAG, "Entering main loop");

    while (1)
    {
        ei_impulse_result_t result = edge_impulse_run();
        stateMachine.determineCommand(result);
    }
}