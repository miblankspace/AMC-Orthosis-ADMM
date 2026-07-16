#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"

#include "microphone.h"
#include "edge_impulse.h"

#include "MotionController.h"
#include "state_machine.h"

#include "MotorCAN.h"


static const char* TAG="Main";


static uint32_t millis()
{
    return esp_timer_get_time()/1000;
}



extern "C" void app_main()
{

    init_i2s();
    start_mic_task();

    edge_impulse_init();



    MotorCAN motor(
        Config::kCanTxPin,
        Config::kCanRxPin,
        Config::kMotorId
    );


    MotionController motion(
        Config::kMaxErpm,
        Config::kAccelStep,
        Config::kStopStep,
        Config::kEmergencyStep,
        Config::kMotionUpdatePeriodMs
    );


    StateMachine stateMachine(motion);



    ESP_LOGI(TAG,"Starting");



    if(!motor.begin())
    {
        ESP_LOGE(TAG,"CAN failed");

        while(true)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }



    while(1)
    {

        motor.poll();


        // ei_impulse_result_t result =
        //     edge_impulse_run();


        //stateMachine.determineCommand(result);



        motion.update();



        if(motion.isEmergencyStopActive())
        {
            motor.servoSetDuty(0);
        }
        else
        {
            motor.servoSetRPM(
                motion.getCurrentERPM()
            );
        }



        vTaskDelay(
            pdMS_TO_TICKS(5)
        );
    }
}