#if MJPEG_AVI_EXPERIMENT

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "Display_ST7701.h"
#include "TCA9554PWR.h"
#include "Touch_CST820.h"

extern "C" {
#include "avi_player.h"
#include "jpeg_decoder.h"
}

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

namespace {
constexpr int kSdClkPin = 2;
constexpr int kSdCmdPin = 1;
constexpr int kSdD0Pin = 42;

constexpr const char* kVideoCandidates[] = {
    "/sdcard/media/video/video.avi",
    "/sdcard/videos/sample.avi",
};

constexpr bool kSwapColorBytes = false;

static const char* TAG = "mjpeg_avi";
static EventGroupHandle_t sVideoEvents = nullptr;
static constexpr EventBits_t kVideoDoneBit = BIT0;

typedef struct {
  esp_lcd_panel_handle_t lcd;
  int screen_w;
  int screen_h;
  uint8_t* outbuf;
  size_t out_sz;
  uint8_t* work;
  size_t work_cap;
  uint32_t fps_frames;
  int64_t fps_start_us;
} mjpeg_draw_ctx_t;

// Default JPEG Huffman table injected for MJPEG streams that omit DHT.
static const uint8_t s_default_dht[] = {
    0xFF,0xC4,0x01,0xA2,
    0x00,0x00,0x01,0x05,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,
    0x01,0x00,0x03,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
    0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,
    0x10,0x00,0x02,0x01,0x03,0x03,0x02,0x04,0x03,0x05,0x05,0x04,0x04,0x00,0x00,0x01,
    0x7D,0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
    0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,0xF0,0x24,
    0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,0x28,0x29,0x2A,
    0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x53,0x54,
    0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x73,0x74,0x75,
    0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x92,0x93,0x94,0x95,
    0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,
    0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,
    0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,
    0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,
    0x11,0x00,0x02,0x01,0x02,0x04,0x04,0x03,0x04,0x07,0x05,0x04,0x04,0x00,0x01,0x02,
    0x77,0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
    0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xA1,0xB1,0xC1,0x09,0x23,0x33,0x52,0xF0,0x15,
    0x62,0x72,0xD1,0x0A,0x16,0x24,0x34,0xE1,0x25,0xF1,0x17,0x18,0x19,0x1A,0x26,0x27,0x28,
    0x29,0x2A,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x53,
    0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x73,0x74,
    0x75,0x76,0x77,0x78,0x79,0x7A,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x92,0x93,
    0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,
    0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,
    0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,
    0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA
};

bool initSdCard() {
  if (!SD_MMC.setPins(kSdClkPin, kSdCmdPin, kSdD0Pin, -1, -1, -1)) {
    ESP_LOGE(TAG, "SD: setPins failed");
    return false;
  }
  Set_EXIO(EXIO_PIN4, High);
  delay(10);
  if (!SD_MMC.begin("/sdcard", true, true)) {
    ESP_LOGE(TAG, "SD: begin failed");
    return false;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    ESP_LOGE(TAG, "SD: no card");
    return false;
  }
  ESP_LOGI(TAG, "SD mounted: total=%llu used=%llu", SD_MMC.totalBytes(), SD_MMC.usedBytes());
  return true;
}

bool fileExistsPosix(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) return false;
  fclose(f);
  return true;
}

const char* findVideoPath() {
  for (const char* path : kVideoCandidates) {
    if (fileExistsPosix(path)) {
      return path;
    }
  }
  return nullptr;
}

inline bool jpegHasDht(const uint8_t* p, size_t n) {
  for (size_t i = 0; i + 1 < n; ++i) {
    if (p[i] == 0xFF && p[i + 1] == 0xC4) return true;
  }
  return false;
}

inline int findMarker(const uint8_t* p, size_t n, uint8_t markerLow) {
  for (size_t i = 0; i + 1 < n; ++i) {
    if (p[i] == 0xFF && p[i + 1] == markerLow) return static_cast<int>(i);
  }
  return -1;
}

const uint8_t* prepareMjpegBitstream(mjpeg_draw_ctx_t* ctx, const uint8_t* in,
                                     size_t in_sz, size_t* out_sz) {
  *out_sz = in_sz;
  if (in_sz < 2 || in[0] != 0xFF || in[1] != 0xD8) return nullptr;
  if (jpegHasDht(in, in_sz)) return in;

  const int sos = findMarker(in, in_sz, 0xDA);
  if (sos < 0) return nullptr;

  const size_t need = in_sz + sizeof(s_default_dht);
  if (ctx->work_cap < need) {
    uint8_t* nbuf = static_cast<uint8_t*>(
        heap_caps_realloc(ctx->work, need, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!nbuf) return nullptr;
    ctx->work = nbuf;
    ctx->work_cap = need;
  }

  memcpy(ctx->work, in, static_cast<size_t>(sos));
  memcpy(ctx->work + sos, s_default_dht, sizeof(s_default_dht));
  memcpy(ctx->work + sos + sizeof(s_default_dht), in + sos,
         in_sz - static_cast<size_t>(sos));
  *out_sz = need;
  return ctx->work;
}

void logCallbackFps(mjpeg_draw_ctx_t* ctx) {
  const int64_t nowUs = esp_timer_get_time();
  if (ctx->fps_start_us == 0) ctx->fps_start_us = nowUs;
  ++ctx->fps_frames;
  const int64_t elapsedUs = nowUs - ctx->fps_start_us;
  if (elapsedUs < 1000000) return;
  const float fps = (1000000.0f * static_cast<float>(ctx->fps_frames)) /
                    static_cast<float>(elapsedUs);
  ESP_LOGI(TAG, "mjpeg decode/draw fps: %.2f", fps);
  ctx->fps_frames = 0;
  ctx->fps_start_us = nowUs;
}

void drawCenteredRgb565(esp_lcd_panel_handle_t lcd, int sw, int sh, const uint8_t* rgb565,
                        int w, int h) {
  if (!lcd || !rgb565 || w <= 0 || h <= 0) return;
  int x0 = (sw - w) / 2;
  int y0 = (sh - h) / 2;
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  int x1 = x0 + w;
  int y1 = y0 + h;
  if (x1 > sw) x1 = sw;
  if (y1 > sh) y1 = sh;
  esp_lcd_panel_draw_bitmap(lcd, x0, y0, x1, y1, rgb565);
}

void onVideoFrame(frame_data_t* frame, void* arg) {
  auto* ctx = static_cast<mjpeg_draw_ctx_t*>(arg);
  if (!ctx || !frame || frame->type != FRAME_TYPE_VIDEO) return;
  if (frame->video_info.frame_format != FORMAT_MJEPG) return;

  size_t prepped_sz = 0;
  const uint8_t* prepped =
      prepareMjpegBitstream(ctx, frame->data, frame->data_bytes, &prepped_sz);
  if (!prepped) return;

  esp_jpeg_image_cfg_t jpeg_cfg = {
      .indata = const_cast<uint8_t*>(prepped),
      .indata_size = prepped_sz,
      .outbuf = ctx->outbuf,
      .outbuf_size = ctx->out_sz,
      .out_format = JPEG_IMAGE_FORMAT_RGB565,
  };
  jpeg_cfg.flags.swap_color_bytes = kSwapColorBytes;

  esp_jpeg_image_output_t out = {};
  if (esp_jpeg_decode(&jpeg_cfg, &out) != ESP_OK) {
    return;
  }

  drawCenteredRgb565(ctx->lcd, ctx->screen_w, ctx->screen_h, ctx->outbuf,
                     static_cast<int>(out.width), static_cast<int>(out.height));
  logCallbackFps(ctx);
}

void onAviPlayEnd(void* user_data) {
  (void)user_data;
  if (sVideoEvents) {
    xEventGroupSetBits(sVideoEvents, kVideoDoneBit);
  }
}

void playerTask(void*) {
#if CONFIG_PM_ENABLE
  esp_pm_lock_handle_t pm_lock = nullptr;
  if (esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "mjpeg", &pm_lock) == ESP_OK) {
    (void)esp_pm_lock_acquire(pm_lock);
  }
#endif

  const char* path = findVideoPath();
  if (!path) {
    ESP_LOGE(TAG, "No AVI file found. Tried /sdcard/media/video/video.avi and /sdcard/videos/sample.avi");
    vTaskDelete(nullptr);
    return;
  }

  const int screen_w = 480;
  const int screen_h = 480;
  const size_t out_sz = static_cast<size_t>(screen_w) * screen_h * 2;
  uint8_t* outbuf = static_cast<uint8_t*>(
      heap_caps_malloc(out_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!outbuf) {
    ESP_LOGE(TAG, "No PSRAM for JPEG output buffer");
    vTaskDelete(nullptr);
    return;
  }

  static mjpeg_draw_ctx_t ctx = {};
  ctx.lcd = panel_handle;
  ctx.screen_w = screen_w;
  ctx.screen_h = screen_h;
  ctx.outbuf = outbuf;
  ctx.out_sz = out_sz;
  ctx.work = nullptr;
  ctx.work_cap = 0;
  ctx.fps_frames = 0;
  ctx.fps_start_us = 0;

  sVideoEvents = xEventGroupCreate();
  if (!sVideoEvents) {
    ESP_LOGE(TAG, "Event group alloc failed");
    vTaskDelete(nullptr);
    return;
  }

  avi_player_config_t cfg = {
      .buffer_size = 512 * 1024,
      .video_cb = onVideoFrame,
      .audio_cb = nullptr,
      .audio_set_clock_cb = nullptr,
      .avi_play_end_cb = onAviPlayEnd,
      .priority = 10,
      .coreID = 1,
      .user_data = &ctx,
      .stack_size = 8192,
      .stack_in_psram = true,
  };

  avi_player_handle_t player = nullptr;
  esp_err_t err = avi_player_init(cfg, &player);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "avi_player_init failed: %d", static_cast<int>(err));
    vTaskDelete(nullptr);
    return;
  }

  ESP_LOGI(TAG, "MJPEG AVI start: %s", path);
  ESP_LOGI(TAG, "JPEG swap_color_bytes=%d", kSwapColorBytes ? 1 : 0);
  while (true) {
    xEventGroupClearBits(sVideoEvents, kVideoDoneBit);
    ctx.fps_frames = 0;
    ctx.fps_start_us = 0;
    err = avi_player_play_from_file(player, path);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "avi_player_play_from_file failed: %d", static_cast<int>(err));
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }
    xEventGroupWaitBits(sVideoEvents, kVideoDoneBit, pdTRUE, pdFALSE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 MJPEG AVI Experiment");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(100);

  TCA9554PWR_Init(0x00);
  Set_EXIO(EXIO_PIN8, Low);
  delay(10);

  LCD_Init();
  if (!panel_handle) {
    ESP_LOGE(TAG, "panel_handle is null after LCD_Init");
    return;
  }

  if (!initSdCard()) {
    ESP_LOGE(TAG, "SD init failed");
    return;
  }

  xTaskCreatePinnedToCore(playerTask, "mjpeg_player", 8192, nullptr, 5, nullptr, 0);
}

void loop() {
  delay(20);
}

#endif  // MJPEG_AVI_EXPERIMENT
