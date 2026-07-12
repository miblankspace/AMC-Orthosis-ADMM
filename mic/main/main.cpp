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
    init_i2s();
}