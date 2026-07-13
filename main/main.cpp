#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "microphone.h"
#include "edge_impulse.h"

#define AUDIO_BUFFER_SIZE 256
int16_t audio_buffer[AUDIO_BUFFER_SIZE];

extern "C" void app_main()
{
    // initialize mic
    init_i2s();

    while (1)
    {
        // 1. collect samples

        // 2. run classifier

        // 3. pass to state machine
    }
}

void determine_command(string )