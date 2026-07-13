#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

static float dummy_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

static int get_signal_data(size_t offset, size_t length, float *out_ptr)
{
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = dummy_buffer[offset + i];
    }
    return EIDSP_OK;
}

extern "C" void app_main(void)
{
    printf("Generating dummy audio. Frame size = %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);

    // Fill with a simple sine wave, not silence, so DSP has something to chew on
    for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i++) {
        dummy_buffer[i] = 1000.0f * sinf(2.0f * 3.14159f * 440.0f * i / 16000.0f);
    }

    while (1) {
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        signal.get_data = &get_signal_data;

        ei_impulse_result_t result;
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res != EI_IMPULSE_OK) {
            printf("run_classifier failed (%d)\n", res);
        } else {
            printf("Predictions (DSP: %d ms, Classify: %d ms):\n",
                   result.timing.dsp, result.timing.classification);

            for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                printf("  %s: %.2f\n",
                       ei_classifier_inferencing_categories[i],
                       result.classification[i].value);
            }
        }
        printf("---\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}