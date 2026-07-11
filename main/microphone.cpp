#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include "microphone.h"

#define I2S_BCLK 15
#define I2S_WS 17
#define I2S_DIN 16

#define SAMPLE_RATE 16000

i2s_chan_handle_t rx_chan;

// mic config
void init_i2s()
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    // create channel
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t std_cfg =
    {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
        {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_15,
            .ws = GPIO_NUM_17,
            .dout = I2S_GPIO_UNUSED,
            .din = GPIO_NUM_16,
            .invert_flags =
            {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    // initialize and enable channel
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

static int32_t dc_estimate = 0;

// collect samples, write into buffer, return # of samples collected
int read_input(int16_t *buffer, int buf_len)
{
    int32_t samples[buf_len * 2];
    size_t bytes_read = 0;

    ESP_ERROR_CHECK(i2s_channel_read(rx_chan, samples, sizeof(samples), &bytes_read, portMAX_DELAY));

    // stereo = left + right 32-bit samples
    int frames = bytes_read / 8;
    int output_samples = 0;

    for(int i = 0; i < frames && output_samples < buffer_len; i++)
    {
        // SEL connected to GND (left channel) by default
        int32_t raw = samples[i * 2];

        // SPH6045 produces 18-bit signed data, shift bits
        int32_t audio = raw >> 14;

        // sign extension
        if(audio & 0x20000)
        {
            audio |= 0xFFFC0000;
        }

        // estimate and remove DC offset with slow moving average
        dc_estimate += (audio - dc_estimate) >> 8;
        int32_t filtered = audio - dc_estimate;

        // convert to 16-bit PCM for processing
        buffer[output_samples] = filtered >> 2;

        output_samples++;
    }

    return output_samples;
}