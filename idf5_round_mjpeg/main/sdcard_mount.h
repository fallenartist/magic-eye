#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sdcard_mount(const char* base_path);
void sdcard_unmount(const char* base_path);

#ifdef __cplusplus
}
#endif
