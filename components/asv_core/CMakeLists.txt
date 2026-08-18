set(ASV_CORE_SRCS
    "src/application/ros_node.cpp"
    "src/application/asv_control_app.cpp"
    "src/communication/esp32_serial_transport.c"
    "src/control/RLPID_controller.cpp"
    "src/estimator/estimator.cpp"
    "src/iq_math_extension/iq_scalar.cpp"
)

idf_component_register(
    SRCS ${ASV_CORE_SRCS}
    INCLUDE_DIRS "include"
    REQUIRES
        "micro_ros_espidf_component"
        "iqmath"
    PRIV_REQUIRES
        "esp_driver_uart"
        "esp_driver_gpio"
        "esp_timer"
)

target_compile_definitions(
    ${COMPONENT_LIB}
    PRIVATE GLOBAL_IQ=20
)
