#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include "esp_log.h"
#include "microphone.h"

static const char* TAG = "Microphone";

#define I2S_BCLK 15
#define I2S_WS 17
#define I2S_DIN 16

#define SAMPLE_RATE 16000
#define MIC_MAX_SAMPLES 4000
#define QUEUE_DEPTH 3

static i2s_chan_handle_t rx_chan;
static QueueHandle_t mic_queue; // holds EI_CLASSIFIER_SIZE int16_t samples per item


static int32_t dc_estimate = 0;
static int32_t samples[MIC_MAX_SAMPLES * 2];

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

    mic_queue = xQueueCreate(QUEUE_DEPTH, EI_CLASSIFIER_SLICE_SIZE * sizeof(int16_t));
    if (!mic_queue)
    {
        ESP_LOGE(TAG, "Failed to create mic queue");
    }
}

// runs forever on its own task, signal processing, pushes into mic_queue to be read by EI
static void mic_task(void* arg)
{
    static int16_t slice[EI_CLASSIFIER_SLICE_SIZE];
    int slice_pos = 0;

    while(true)
    {
        size_t bytes_read = 0;
        ESP_ERROR_CHECK(i2s_channel_read(rx_chan, samples, sizeof(samples), &bytes_read, portMAX_DELAY));

        // stereo = left + right 32-bit samples
        int frames = bytes_read / 8;

        for(int i = 0; i < frames; i++)
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
            slice[slice_pos++] = filtered >> 2;

            if (slice_pos == EI_CLASSIFIER_SLICE_SIZE)
            {
                if (xQueueSend(mic_queue, slice, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Mic queue full, dropped a slice");
                }
                slice_pos = 0;
            }
        }
    }
}

void start_mic_task()
{
    xTaskCreatePinnedToCore(mic_task, "mic_task", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}

int mic_read(int16_t *buffer, uint32_t timeout_ms)
{
    if (xQueueReceive(mic_queue, buffer, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
    {
        return EI_CLASSIFIER_SLICE_SIZE;
    }
    return 0;
}