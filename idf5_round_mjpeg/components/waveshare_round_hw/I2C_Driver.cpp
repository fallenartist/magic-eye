#include "I2C_Driver.h"

#include <string.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace {
constexpr i2c_port_t kI2CPort = I2C_NUM_0;
bool s_i2c_initialized = false;
}

void I2C_Init(void) {
  if (s_i2c_initialized) {
    return;
  }

  i2c_config_t cfg = {};
  cfg.mode = I2C_MODE_MASTER;
  cfg.sda_io_num = static_cast<gpio_num_t>(I2C_SDA_PIN);
  cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.scl_io_num = static_cast<gpio_num_t>(I2C_SCL_PIN);
  cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.master.clk_speed = 400000;

  if (i2c_param_config(kI2CPort, &cfg) != ESP_OK) {
    printf("I2C param config failed\r\n");
    return;
  }

  esp_err_t err = i2c_driver_install(kI2CPort, I2C_MODE_MASTER, 0, 0, 0);
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
    s_i2c_initialized = true;
    return;
  }

  printf("I2C driver install failed: %d\r\n", (int)err);
}

bool I2C_Read(uint8_t Driver_addr, uint8_t Reg_addr, uint8_t *Reg_data, uint32_t Length)
{
  if (!s_i2c_initialized) {
    I2C_Init();
  }
  if (!s_i2c_initialized) {
    return true;
  }

  esp_err_t err = i2c_master_write_read_device(
      kI2CPort, Driver_addr, &Reg_addr, 1, Reg_data, Length, pdMS_TO_TICKS(100));
  if (err != ESP_OK) {
    printf("The I2C transmission fails. - I2C Read\r\n");
    return true;
  }
  return false;
}

bool I2C_Write(uint8_t Driver_addr, uint8_t Reg_addr, const uint8_t *Reg_data, uint32_t Length)
{
  if (!s_i2c_initialized) {
    I2C_Init();
  }
  if (!s_i2c_initialized) {
    return true;
  }

  uint8_t buf[1 + 32];
  if (Length > 32) {
    printf("I2C write too long: %lu\r\n", (unsigned long)Length);
    return true;
  }
  buf[0] = Reg_addr;
  if (Length > 0 && Reg_data) {
    memcpy(&buf[1], Reg_data, Length);
  }

  esp_err_t err =
      i2c_master_write_to_device(kI2CPort, Driver_addr, buf, Length + 1, pdMS_TO_TICKS(100));
  if (err != ESP_OK)
  {
    printf("The I2C transmission fails. - I2C Write\r\n");
    return true;
  }
  return false;
}
