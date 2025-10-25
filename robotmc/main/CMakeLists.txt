idf_component_register(
    SRCS "app_main.cpp"
    REQUIRES esp_wifi
    REQUIRES esp_event
    REQUIRES esp_driver_tsens
    REQUIRES esp_timer
    REQUIRES esp_driver_mcpwm
    REQUIRES esp_driver_gpio
    REQUIRES esp_driver_rmt
    INCLUDE_DIRS "./include"
    INCLUDE_DIRS "../../common"
)
