#pragma once

#include <stdint.h>
#include "model-parameters/model_metadata.h"

void init_i2s();
void start_mic_task();
int mic_read(int16_t *buffer, uint32_t timeout_ms);