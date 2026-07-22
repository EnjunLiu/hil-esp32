file(GLOB_RECURSE APP_SRCS "../src/*.cpp" "../src/*.c")

idf_component_register(
    SRCS "main.cpp" ${APP_SRCS}
    INCLUDE_DIRS "../include"
    REQUIRES "micro_ros_espidf_component"
    PRIV_REQUIRES "esp_driver_uart" "esp_driver_gpio" "esp_timer"
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE GLOBAL_IQ=20)