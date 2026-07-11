#ifndef MICROPHONE_H
#define MICROPHONE_H

#include <stdint.h>

void init_i2s();

int read_input(int16_t *buffer, int buf_len);

#endif