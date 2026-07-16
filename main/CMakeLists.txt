idf_component_register(
    SRCS "main.cpp" "microphone/microphone.cpp" "classifier/edge_impulse.cpp"
    INCLUDE_DIRS "." "microphone" "state_machine" "enums" "classifier"
    REQUIRES edge_impulse "esp_driver_i2s"
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    EIDSP_USE_ESP_DSP=0
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-nonnull
    -Wno-missing-field-initializers
    -Wno-deprecated-declarations
    -Wno-error
)