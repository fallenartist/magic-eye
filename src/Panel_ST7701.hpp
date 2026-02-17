#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include <driver/i2c.h>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>

class LGFX : public lgfx::LGFX_Device {
 public:
  lgfx::Bus_RGB _bus_instance;
  lgfx::Panel_ST7701 _panel_instance;
  lgfx::Touch_CST816S _touch_instance;
  lgfx::Light_PWM _light_instance;

  LGFX(void) {
    {
      auto cfg = _panel_instance.config();
      cfg.memory_width = 480;
      cfg.memory_height = 480;
      cfg.panel_width = 480;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      _panel_instance.config(cfg);
    }

    {
      auto cfg = _panel_instance.config_detail();
      cfg.pin_cs = -1;     // CS is handled via EXIO on this board.
      cfg.pin_sclk = 2;
      cfg.pin_mosi = 1;
      _panel_instance.config_detail(cfg);
    }

    {
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;

      cfg.pin_d0 = GPIO_NUM_5;    // B0
      cfg.pin_d1 = GPIO_NUM_45;   // B1
      cfg.pin_d2 = GPIO_NUM_48;   // B2
      cfg.pin_d3 = GPIO_NUM_47;   // B3
      cfg.pin_d4 = GPIO_NUM_21;   // B4
      cfg.pin_d5 = GPIO_NUM_14;   // G0
      cfg.pin_d6 = GPIO_NUM_13;   // G1
      cfg.pin_d7 = GPIO_NUM_12;   // G2
      cfg.pin_d8 = GPIO_NUM_11;   // G3
      cfg.pin_d9 = GPIO_NUM_10;   // G4
      cfg.pin_d10 = GPIO_NUM_9;   // G5
      cfg.pin_d11 = GPIO_NUM_46;  // R0
      cfg.pin_d12 = GPIO_NUM_3;   // R1
      cfg.pin_d13 = GPIO_NUM_8;   // R2
      cfg.pin_d14 = GPIO_NUM_18;  // R3
      cfg.pin_d15 = GPIO_NUM_17;  // R4

      cfg.pin_henable = GPIO_NUM_40;  // DE
      cfg.pin_vsync = GPIO_NUM_39;
      cfg.pin_hsync = GPIO_NUM_38;
      cfg.pin_pclk = GPIO_NUM_41;
      cfg.freq_write = 14000000;

      cfg.hsync_polarity = 0;
      cfg.hsync_front_porch = 50;
      cfg.hsync_pulse_width = 8;
      cfg.hsync_back_porch = 8;
      cfg.vsync_polarity = 0;
      cfg.vsync_front_porch = 8;
      cfg.vsync_pulse_width = 3;
      cfg.vsync_back_porch = 8;
      cfg.pclk_idle_high = 0;
      cfg.de_idle_high = 1;

      _bus_instance.config(cfg);
    }
    _panel_instance.setBus(&_bus_instance);

    {
      auto cfg = _touch_instance.config();
      cfg.x_min = 0;
      cfg.x_max = 480;
      cfg.y_min = 0;
      cfg.y_max = 480;
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;

      cfg.i2c_port = I2C_NUM_1;
      cfg.pin_int = 16;
      cfg.pin_sda = 15;
      cfg.pin_scl = 7;
      cfg.pin_rst = -1;

      cfg.freq = 400000;
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = GPIO_NUM_6;
      cfg.freq = 20000;
      _light_instance.config(cfg);
    }
    _panel_instance.light(&_light_instance);

    setPanel(&_panel_instance);
  }
};
