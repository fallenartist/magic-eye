// Display_ST7701.h
#pragma once
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "TCA9554PWR.h"
#include "Touch_CST820.h"

#define LCD_CLK_PIN   2
#define LCD_MOSI_PIN  1 
#define LCD_Backlight_PIN   6 

// Backlight   
#define PWM_Channel     1       // PWM Channel   
#define Frequency       20000   // PWM frequency
#define Resolution      10      // PWM resolution ratio     MAX:13
#define Dutyfactor      500    // PWM Dutyfactor      
#define Backlight_MAX   100     

#ifndef ESP_PANEL_LCD_WIDTH
#define ESP_PANEL_LCD_WIDTH                       (480)
#endif
#ifndef ESP_PANEL_LCD_HEIGHT
#define ESP_PANEL_LCD_HEIGHT                      (480)
#endif
#ifndef ESP_PANEL_LCD_COLOR_BITS
#define ESP_PANEL_LCD_COLOR_BITS                  (16)
#endif
#ifndef ESP_PANEL_LCD_RGB_PIXEL_BITS
#define ESP_PANEL_LCD_RGB_PIXEL_BITS              (16)
#endif
#ifndef ESP_PANEL_LCD_RGB_DATA_WIDTH
#define ESP_PANEL_LCD_RGB_DATA_WIDTH              (16)
#endif
#ifndef ESP_PANEL_LCD_RGB_TIMING_FREQ_HZ
#define ESP_PANEL_LCD_RGB_TIMING_FREQ_HZ          (16 * 1000 * 1000)
#endif
#ifndef ESP_PANEL_LCD_RGB_TIMING_HPW
#define ESP_PANEL_LCD_RGB_TIMING_HPW              (8)
#endif
#ifndef ESP_PANEL_LCD_RGB_TIMING_HBP
#define ESP_PANEL_LCD_RGB_TIMING_HBP              (10)
#endif
#ifndef ESP_PANEL_LCD_RGB_TIMING_HFP
#define ESP_PANEL_LCD_RGB_TIMING_HFP              (50)
#endif
#ifndef ESP_PANEL_LCD_RGB_TIMING_VPW
#define ESP_PANEL_LCD_RGB_TIMING_VPW              (3)
#endif
#ifndef ESP_PANEL_LCD_RGB_TIMING_VBP
#define ESP_PANEL_LCD_RGB_TIMING_VBP              (8)
#endif
#ifndef ESP_PANEL_LCD_RGB_TIMING_VFP
#define ESP_PANEL_LCD_RGB_TIMING_VFP              (8)
#endif
#ifndef ESP_PANEL_LCD_RGB_FRAME_BUF_NUM
#define ESP_PANEL_LCD_RGB_FRAME_BUF_NUM           (2)
#endif
#ifndef ESP_PANEL_LCD_RGB_BOUNCE_BUF_SIZE
#define ESP_PANEL_LCD_RGB_BOUNCE_BUF_SIZE         (ESP_PANEL_LCD_WIDTH * 10)
#endif

// Pin definitions
#define ESP_PANEL_LCD_PIN_NUM_RGB_HSYNC           (38)
#define ESP_PANEL_LCD_PIN_NUM_RGB_VSYNC           (39)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DE              (40)
#define ESP_PANEL_LCD_PIN_NUM_RGB_PCLK            (41)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA0           (5)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA1           (45)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA2           (48)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA3           (47)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA4           (21)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA5           (14)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA6           (13)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA7           (12)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA8           (11)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA9           (10)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA10          (9)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA11          (46)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA12          (3)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA13          (8)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA14          (18)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA15          (17)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DISP            (-1)

extern uint8_t LCD_Backlight;
extern esp_lcd_panel_handle_t panel_handle;   

void LCD_Init();
void LCD_addWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint8_t* color);
void Backlight_Init();
void Set_Backlight(uint8_t Light);
