cmake_minimum_required(VERSION 3.16)

# Let the ESP-IDF VS Code extension build from the repository root while using
# the pure-IDF MJPEG AVI experiment app kept in idf5_round_mjpeg/main.
#
# Keep EXTRA_COMPONENT_DIRS relative so ESP-IDF doesn't need to split an
# absolute path containing spaces (this repo lives under "FIGLE Work").
set(EXTRA_COMPONENT_DIRS
    "idf5_round_mjpeg/main"
    "idf5_round_mjpeg/components"
)
set(SDKCONFIG_DEFAULTS "idf5_round_mjpeg/sdkconfig.defaults")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(magic_eye_idf5_round_mjpeg)
