#include <stdbool.h>
#include <stdio.h>

#include "driver/sdmmc_host.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdcard_mount.h"
#include "sdmmc_cmd.h"

static const char* TAG = "sd";
static sdmmc_card_t* s_card;
static bool s_mounted;

// Round board SD wiring (1-bit SDMMC)
#define ROUND_SD_CLK_GPIO 2
#define ROUND_SD_CMD_GPIO 1
#define ROUND_SD_D0_GPIO 42

esp_err_t sdcard_mount(const char* base_path) {
  if (s_mounted) return ESP_OK;

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = 40000;

  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.width = 1;
  slot_config.clk = (gpio_num_t)ROUND_SD_CLK_GPIO;
  slot_config.cmd = (gpio_num_t)ROUND_SD_CMD_GPIO;
  slot_config.d0 = (gpio_num_t)ROUND_SD_D0_GPIO;
  slot_config.gpio_cd = (gpio_num_t)-1;
  slot_config.gpio_wp = (gpio_num_t)-1;

  esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
      .format_if_mount_failed = false,
      .max_files = 5,
      .allocation_unit_size = 16 * 1024,
  };

  ESP_LOGI(TAG, "Mounting SDMMC 1-bit: CMD=%d CLK=%d D0=%d", ROUND_SD_CMD_GPIO,
           ROUND_SD_CLK_GPIO, ROUND_SD_D0_GPIO);

  esp_err_t err =
      esp_vfs_fat_sdmmc_mount(base_path, &host, &slot_config, &mount_cfg, &s_card);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_vfs_fat_sdmmc_mount failed: %s", esp_err_to_name(err));
    return err;
  }
  s_mounted = true;
  sdmmc_card_print_info(stdout, s_card);
  return ESP_OK;
}

void sdcard_unmount(const char* base_path) {
  if (!s_mounted) return;
  (void)esp_vfs_fat_sdcard_unmount(base_path, s_card);
  s_card = NULL;
  s_mounted = false;
}
