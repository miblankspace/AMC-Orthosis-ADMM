idf_component_register(
    SRCS "main.cpp" "microphone/microphone.cpp" "classifier/edge_impulse.cpp" "state_machine/state_machine.cpp"
        "ak40_can_velocity/MotionController.cpp" "ak40_can_velocity/MotorCAN.cpp" "ak40_can_velocity/Status.cpp"
    INCLUDE_DIRS "." "microphone" "state_machine" "classifier" "ak40_can_velocity"
    REQUIRES edge_impulse "esp_driver_i2s" esp_timer esp_driver_gpio driver
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