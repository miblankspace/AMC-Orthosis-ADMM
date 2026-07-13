#include <stdio.h>
#include <stdint.h>
#include "edge_impulse.h"

void send_audio_data(int16_t *buffer, int samples)
{
    for(int i = 0; i < samples; i++)
    {
        printf("%d\n", buffer[i]);
    }
}