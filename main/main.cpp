#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "microphone.h"


extern "C"
void app_main()
{
    init_i2s();

    while(1)
    {
        
    }
}