file(GLOB_RECURSE EI_SOURCES
    "edge-impulse-sdk/CMSIS/**/*.c"
    "edge-impulse-sdk/dsp/**/*.cpp"
    "edge-impulse-sdk/classifier/**/*.cpp"
    "edge-impulse-sdk/anomaly/**/*.cpp"
    "edge-impulse-sdk/tensorflow/**/*.cc"
    "edge-impulse-sdk/tensorflow/**/*.c"
    "edge-impulse-sdk/porting/espressif/*.cpp"
    "tflite-model/*.cpp"
    "model-parameters/*.cpp"
)

idf_component_register(
    SRCS ${EI_SOURCES}
    INCLUDE_DIRS "." "edge-impulse-sdk" "model-parameters" "tflite-model"
    PRIV_REQUIRES esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    EIDSP_USE_ESP_DSP=0
    TF_LITE_DISABLE_X86_NEON=1
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-nonnull
    -Wno-missing-field-initializers
    -Wno-deprecated-declarations
    -Wno-error
)