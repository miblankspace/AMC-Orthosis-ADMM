#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "edge_impulse.h"
#include "microphone.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

static const char* TAG = "EdgeImpulse";

// double buffer for continuous inference
typedef struct
{
    int16_t buffers[2][EI_CLASSIFIER_SLICE_SIZE];

    volatile uint8_t buf_select;
    volatile uint32_t buf_count;
    volatile bool buf_ready;

    uint32_t n_samples;

} inference_t;

static inference_t inference;

static int get_signal_data(size_t offset, size_t length, float *out_ptr);

void edge_impulse_init()
{
    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.buf_ready = false;

    inference.n_samples = EI_CLASSIFIER_SLICE_SIZE;

    // Required before run_classifier_continuous()
    run_classifier_init();
}

// give samples from part of buffer
static int get_signal_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(
        &inference.buffers[inference.buf_select ^ 1][offset],
        out_ptr,
        length
    );

    return 0;
}

ei_impulse_result_t edge_impulse_run()
{
    ei_impulse_result_t result = {};

    // Non-blocking: never stall the motor/serial loop waiting for audio.
    int samples_read = mic_read(inference.buffers[inference.buf_select], 0);
    if (samples_read != EI_CLASSIFIER_SLICE_SIZE)
    {
        // not enough samples yet, nothing to classify
        return result;
    }

    // switch buffers
    inference.buf_select ^= 1;

    // run classifier
    signal_t signal;

    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = get_signal_data;

    EI_IMPULSE_ERROR res = run_classifier_continuous(
        &signal,
        &result,
        false
    );

    if(res != EI_IMPULSE_OK)
    {
        printf("Classifier failed\n");
        return result;
    }

    return result;
}