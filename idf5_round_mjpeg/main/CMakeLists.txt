idf_component_register(
    SRCS
        "app_main.cpp"
        "sdcard_mount.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        avi_player
        driver
        esp_lcd
        esp_jpeg
        esp_timer
        fatfs
        sdmmc
        waveshare_round_hw
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE MJPEG_AVI_EXPERIMENT=1)
