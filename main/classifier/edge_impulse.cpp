#include <stdio.h>
#include <stdint.h>
#include "edge_impulse.h"
#include "microphone.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "model-parameters/model_metadata.h"

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

void edge_impulse_run()
{
    // fill one slice from microphone
    int samples_read = read_input(
        inference.buffers[inference.buf_select],
        EI_CLASSIFIER_SLICE_SIZE
    );


    if(samples_read != EI_CLASSIFIER_SLICE_SIZE)
    {
        return;
    }

    // switch buffers
    inference.buf_select ^= 1;

    // run classifier
    signal_t signal;

    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = get_signal_data;

    ei_impulse_result_t result;

    EI_IMPULSE_ERROR res = run_classifier_continuous(
        &signal,
        &result,
        false
    );

    if(res != EI_IMPULSE_OK)
    {
        printf("Classifier failed\n");
        return;
    }

    for(size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
    {
        if(strcmp(result.classification[i].label, "noise") != 0 && 
        strcmp(result.classification[i].label, "unknown") != 0 && 
        result.classification[i].value > 0.75)
        {
            printf("Detected: %s", result.classification[i].label);
        }
    }

    printf("\n");
}