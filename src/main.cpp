#include <Arduino.h>

#include "DisplayCanvas.h"
#include "Display_ST7701.h"
#include "MediaDemo.h"
#include "TCA9554PWR.h"
#include "Touch_CST820.h"

#ifndef DISPLAY_CANVAS_PREFER_PANEL_FB
#define DISPLAY_CANVAS_PREFER_PANEL_FB 1
#endif

static DisplayCanvas canvas;
static MediaDemo mediaDemo;

static uint32_t lastTouchPollMs = 0;
static bool touchDown = false;

void IRAM_ATTR touchISR() { Touch_CST820_ISR(); }

void processTouch() {
  const uint32_t now = millis();
  if (now - lastTouchPollMs < 16) {
    return;
  }
  lastTouchPollMs = now;

  Touch_Read_Data();
  const bool pressed = (touch_data.points > 0);
  if (pressed && !touchDown) {
    touchDown = true;
    mediaDemo.advanceMode();
  } else if (!pressed) {
    touchDown = false;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 Media Demo");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(100);

  TCA9554PWR_Init(0x00);
  Set_EXIO(EXIO_PIN8, Low);
  delay(10);

  LCD_Init();
  if (!canvas.begin(DISPLAY_CANVAS_PREFER_PANEL_FB != 0)) {
    Serial.println("Framebuffer alloc failed");
    while (true) {
      delay(1000);
    }
  }
  Serial.printf("Display backend: %s\n", canvas.backendName());

  pinMode(CST820_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CST820_INT_PIN), touchISR, FALLING);

  if (!mediaDemo.begin(&canvas)) {
    Serial.println("SD media init failed. Insert SD card with /media files.");
  } else {
    Serial.println("SD media ready.");
  }

  Serial.println("Tap screen to cycle modes:");
  Serial.println("full_image -> image_sequence -> round_image -> alpha_overlay -> video");
  mediaDemo.update(millis());
}

void loop() {
  processTouch();
  mediaDemo.update(millis());
  delay(1);
}
