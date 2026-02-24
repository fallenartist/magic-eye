#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cctype>
#include <string>
#include <vector>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#include "Display_ST7701.h"
#include "I2C_Driver.h"
#include "TCA9554PWR.h"
#include "sdcard_mount.h"

extern "C" {
#include "avi_player.h"
#include "jpeg_decoder.h"
}

namespace {
constexpr const char* TAG = "round_mjpeg_idf5";
constexpr const char* kMountPoint = "/sdcard";
constexpr const char* kFallbackVideoCandidates[] = {
    "/sdcard/media/video/video.avi",
    "/sdcard/videos/sample.avi",
};
constexpr const char* kRandomClipDirs[] = {
    "/sdcard/media/video/random1s",
    "/sdcard/media/video/random",
    "/sdcard/videos/random1s",
};
constexpr const char* kDoorBellCandidates[] = {
    "/sdcard/images/door-bell.a856",
    "/sdcard/media/images/door-bell.a856",
};

constexpr bool kSwapColorBytes = false;
constexpr size_t kAviPlayerBufferSize = 512 * 1024;
constexpr size_t kJpegWorkingBufferSize = 70 * 1024;
constexpr uint16_t kColorBlue = 0x001F;
constexpr uint16_t kColorBlack = 0x0000;
constexpr uint16_t kColorYellow = 0xFFE0;
constexpr uint16_t kColorCyan = 0x07FF;
constexpr uint16_t kColorMagenta = 0xF81F;
constexpr uint16_t kColorRed = 0xF800;
constexpr TickType_t kPollSliceTicks = pdMS_TO_TICKS(20);
constexpr int64_t kTouchToggleDebounceUs = 250000;

constexpr uint8_t kCst820Addr = 0x15;
constexpr uint8_t kCst820RegGesture = 0x01;
constexpr uint8_t kCst820RegVersion = 0x15;
constexpr uint8_t kCst820RegChipId = 0xA7;
constexpr uint8_t kCst820RegFwVersion = 0xA9;
constexpr uint8_t kCst820RegDisAutoSleep = 0xFE;
constexpr gpio_num_t kCst820IntPin = GPIO_NUM_16;

static EventGroupHandle_t s_video_events = nullptr;
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
  uint64_t prep_us_accum;
  uint64_t decode_us_accum;
  uint64_t draw_us_accum;
  uint64_t total_us_accum;
  uint64_t src_bytes_accum;
  uint64_t prepped_bytes_accum;
  volatile bool render_enabled;
  uint8_t* jpeg_workbuf;
  size_t jpeg_workbuf_sz;
  uint16_t* still_canvas;
  size_t still_canvas_sz;
} mjpeg_draw_ctx_t;

typedef struct {
  bool initialized;
  bool down_prev;
  int64_t last_toggle_us;
} touch_state_t;

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

bool fileExistsPosix(const char* path) {
  struct stat st = {};
  return path && stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

bool hasAviSuffix(const char* name) {
  if (!name) return false;
  const char* dot = strrchr(name, '.');
  if (!dot) return false;
  return strcasecmp(dot, ".avi") == 0;
}

std::vector<std::string> scanAviFilesInDir(const char* dir_path) {
  std::vector<std::string> out;
  if (!dir_path) return out;
  DIR* dir = opendir(dir_path);
  if (!dir) return out;

  struct dirent* ent = nullptr;
  while ((ent = readdir(dir)) != nullptr) {
    if (ent->d_name[0] == '.') continue;
    if (!hasAviSuffix(ent->d_name)) continue;
    std::string full = std::string(dir_path) + "/" + ent->d_name;
    if (!fileExistsPosix(full.c_str())) continue;

    // macOS copies to FAT often create AppleDouble "._*" sidecars; FAT may expose
    // their 8.3 aliases (e.g. "_CLIP_~1.AVI"), so validate the AVI RIFF header.
    FILE* f = fopen(full.c_str(), "rb");
    if (!f) continue;
    uint8_t hdr[12] = {0};
    const size_t n = fread(hdr, 1, sizeof(hdr), f);
    fclose(f);
    const bool looks_avi = (n == sizeof(hdr) && memcmp(hdr, "RIFF", 4) == 0 &&
                            memcmp(hdr + 8, "AVI ", 4) == 0);
    if (!looks_avi) {
      ESP_LOGW(TAG, "Skipping non-AVI entry in %s: %s", dir_path, ent->d_name);
      continue;
    }
    out.push_back(std::move(full));
  }
  closedir(dir);
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<std::string> findRandomClipPaths(const char** selected_dir_out = nullptr) {
  for (const char* dir : kRandomClipDirs) {
    auto clips = scanAviFilesInDir(dir);
    if (!clips.empty()) {
      if (selected_dir_out) *selected_dir_out = dir;
      return clips;
    }
  }
  if (selected_dir_out) *selected_dir_out = nullptr;
  return {};
}

const char* findFallbackVideoPath() {
  for (const char* path : kFallbackVideoCandidates) {
    if (fileExistsPosix(path)) return path;
  }
  return nullptr;
}

void fillScreenSolidRgb565(esp_lcd_panel_handle_t lcd, int w, int h, uint16_t color565) {
  if (!lcd || w <= 0 || h <= 0) return;
  static uint16_t line[480];
  const int line_w = (w > 480) ? 480 : w;
  for (int x = 0; x < line_w; ++x) line[x] = color565;
  for (int y = 0; y < h; ++y) {
    esp_lcd_panel_draw_bitmap(lcd, 0, y, line_w, y + 1, line);
  }
}

void fillBufferSolidRgb565(uint16_t* buf, int w, int h, uint16_t color565) {
  if (!buf || w <= 0 || h <= 0) return;
  const size_t pixels = static_cast<size_t>(w) * static_cast<size_t>(h);
  for (size_t i = 0; i < pixels; ++i) buf[i] = color565;
}

bool readTouchReg(uint8_t reg, uint8_t* data, uint32_t len) {
  return !I2C_Read(kCst820Addr, reg, data, len);
}

bool writeTouchReg(uint8_t reg, const uint8_t* data, uint32_t len) {
  return !I2C_Write(kCst820Addr, reg, data, len);
}

void resetCst820() {
  Set_EXIO(EXIO_PIN2, Low);
  vTaskDelay(pdMS_TO_TICKS(10));
  Set_EXIO(EXIO_PIN2, High);
  vTaskDelay(pdMS_TO_TICKS(50));
}

void initTouchCst820(touch_state_t* state) {
  if (!state) return;
  *state = {};
  gpio_reset_pin(kCst820IntPin);
  gpio_set_direction(kCst820IntPin, GPIO_MODE_INPUT);
  gpio_pullup_en(kCst820IntPin);

  resetCst820();
  uint8_t wake = 0xFF;  // disable auto sleep
  (void)writeTouchReg(kCst820RegDisAutoSleep, &wake, 1);

  uint8_t ver = 0;
  uint8_t ids[3] = {0};
  if (readTouchReg(kCst820RegVersion, &ver, 1) && readTouchReg(kCst820RegChipId, ids, 3)) {
    ESP_LOGI(TAG, "CST820 touch: ver=0x%02X chip=0x%02X proj=0x%02X fw=0x%02X", ver, ids[0],
             ids[1], ids[2]);
    state->initialized = true;
    return;
  }
  ESP_LOGW(TAG, "CST820 touch init/read failed (touch toggle disabled)");
}

bool readTouchDown(bool* down_out, uint16_t* x_out = nullptr, uint16_t* y_out = nullptr) {
  if (down_out) *down_out = false;
  uint8_t buf[6] = {0};
  if (!readTouchReg(kCst820RegGesture, buf, sizeof(buf))) return false;

  const bool down = buf[1] != 0;
  if (down_out) *down_out = down;
  if (x_out) *x_out = static_cast<uint16_t>(((buf[2] & 0x0F) << 8) | buf[3]);
  if (y_out) *y_out = static_cast<uint16_t>(((buf[4] & 0x0F) << 8) | buf[5]);
  return true;
}

bool pollTouchToggleEvent(touch_state_t* state) {
  if (!state || !state->initialized) return false;

  bool down = false;
  if (!readTouchDown(&down)) return false;

  const bool rising = down && !state->down_prev;
  state->down_prev = down;
  if (!rising) return false;

  const int64_t now_us = esp_timer_get_time();
  if ((now_us - state->last_toggle_us) < kTouchToggleDebounceUs) return false;
  state->last_toggle_us = now_us;
  return true;
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

const uint8_t* prepareMjpegBitstream(mjpeg_draw_ctx_t* ctx, const uint8_t* in, size_t in_sz,
                                     size_t* out_sz) {
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
  memcpy(ctx->work + sos + sizeof(s_default_dht), in + sos, in_sz - static_cast<size_t>(sos));
  *out_sz = need;
  return ctx->work;
}

void drawCenteredRgb565(esp_lcd_panel_handle_t lcd, int sw, int sh, const uint8_t* rgb565, int w,
                        int h) {
  if (!lcd || !rgb565 || w <= 0 || h <= 0) return;
  int x0 = (sw - w) / 2;
  int y0 = (sh - h) / 2;
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  int x1 = x0 + w;
  int y1 = y0 + h;
  if (x1 > sw) x1 = sw;
  if (y1 > sh) y1 = sh;
  if (x0 < x1 && y0 < y1) {
    esp_lcd_panel_draw_bitmap(lcd, x0, y0, x1, y1, rgb565);
  }
}

void logCallbackFps(mjpeg_draw_ctx_t* ctx) {
  const int64_t now_us = esp_timer_get_time();
  if (ctx->fps_start_us == 0) ctx->fps_start_us = now_us;
  ++ctx->fps_frames;
  const int64_t elapsed_us = now_us - ctx->fps_start_us;
  if (elapsed_us < 1000000) return;
  const float fps = (1000000.0f * static_cast<float>(ctx->fps_frames)) /
                    static_cast<float>(elapsed_us);
  const float denom = (ctx->fps_frames > 0) ? static_cast<float>(ctx->fps_frames) : 1.0f;
  const float prep_ms = static_cast<float>(ctx->prep_us_accum) / 1000.0f / denom;
  const float decode_ms = static_cast<float>(ctx->decode_us_accum) / 1000.0f / denom;
  const float draw_ms = static_cast<float>(ctx->draw_us_accum) / 1000.0f / denom;
  const float total_ms = static_cast<float>(ctx->total_us_accum) / 1000.0f / denom;
  const float src_kb = static_cast<float>(ctx->src_bytes_accum) / 1024.0f / denom;
  const float prepped_kb = static_cast<float>(ctx->prepped_bytes_accum) / 1024.0f / denom;
  ESP_LOGI(TAG,
           "fps=%.2f prep=%.1fms decode=%.1fms draw=%.1fms total=%.1fms src=%.1fKB prepped=%.1fKB",
           fps, prep_ms, decode_ms, draw_ms, total_ms, src_kb, prepped_kb);
  ctx->fps_frames = 0;
  ctx->fps_start_us = now_us;
  ctx->prep_us_accum = 0;
  ctx->decode_us_accum = 0;
  ctx->draw_us_accum = 0;
  ctx->total_us_accum = 0;
  ctx->src_bytes_accum = 0;
  ctx->prepped_bytes_accum = 0;
}

void onVideoFrame(frame_data_t* frame, void* arg) {
  auto* ctx = static_cast<mjpeg_draw_ctx_t*>(arg);
  if (!ctx || !frame || frame->type != FRAME_TYPE_VIDEO) return;
  if (frame->video_info.frame_format != FORMAT_MJEPG) return;

  const int64_t t0 = esp_timer_get_time();
  size_t prepped_sz = 0;
  const uint8_t* prepped =
      prepareMjpegBitstream(ctx, frame->data, frame->data_bytes, &prepped_sz);
  const int64_t t1 = esp_timer_get_time();
  if (!prepped) return;

  esp_jpeg_image_cfg_t jpeg_cfg = {
      .indata = const_cast<uint8_t*>(prepped),
      .indata_size = prepped_sz,
      .outbuf = ctx->outbuf,
      .outbuf_size = ctx->out_sz,
      .out_format = JPEG_IMAGE_FORMAT_RGB565,
  };
  jpeg_cfg.out_scale = JPEG_IMAGE_SCALE_0;
  jpeg_cfg.flags.swap_color_bytes = kSwapColorBytes;
  jpeg_cfg.advanced.working_buffer = ctx->jpeg_workbuf;
  jpeg_cfg.advanced.working_buffer_size = ctx->jpeg_workbuf_sz;

  esp_jpeg_image_output_t out = {};
  if (esp_jpeg_decode(&jpeg_cfg, &out) != ESP_OK) return;
  const int64_t t2 = esp_timer_get_time();
  if (!ctx->render_enabled) return;

  drawCenteredRgb565(ctx->lcd, ctx->screen_w, ctx->screen_h, ctx->outbuf,
                     static_cast<int>(out.width), static_cast<int>(out.height));
  const int64_t t3 = esp_timer_get_time();

  ctx->prep_us_accum += static_cast<uint64_t>(t1 - t0);
  ctx->decode_us_accum += static_cast<uint64_t>(t2 - t1);
  ctx->draw_us_accum += static_cast<uint64_t>(t3 - t2);
  ctx->total_us_accum += static_cast<uint64_t>(t3 - t0);
  ctx->src_bytes_accum += frame->data_bytes;
  ctx->prepped_bytes_accum += prepped_sz;
  logCallbackFps(ctx);
}

void onAviPlayEnd(void* user_data) {
  (void)user_data;
  if (s_video_events) xEventGroupSetBits(s_video_events, kVideoDoneBit);
}

bool readBinaryHeader(FILE* f, const char magic[4], uint16_t* w_out, uint16_t* h_out) {
  if (!f || !magic || !w_out || !h_out) return false;
  uint8_t header[8] = {0};
  if (fread(header, 1, sizeof(header), f) != sizeof(header)) return false;
  if (memcmp(header, magic, 4) != 0) return false;
  const uint16_t w = static_cast<uint16_t>(header[4] | (header[5] << 8));
  const uint16_t h = static_cast<uint16_t>(header[6] | (header[7] << 8));
  if (w == 0 || h == 0) return false;
  *w_out = w;
  *h_out = h;
  return true;
}

bool filenameContainsCaseInsensitive(const char* s, const char* needle) {
  if (!s || !needle) return false;
  const size_t ns = strlen(s);
  const size_t nn = strlen(needle);
  if (nn == 0) return true;
  if (ns < nn) return false;
  for (size_t i = 0; i + nn <= ns; ++i) {
    bool match = true;
    for (size_t j = 0; j < nn; ++j) {
      const unsigned char a = static_cast<unsigned char>(s[i + j]);
      const unsigned char b = static_cast<unsigned char>(needle[j]);
      if (std::tolower(a) != std::tolower(b)) {
        match = false;
        break;
      }
    }
    if (match) return true;
  }
  return false;
}

std::string findDoorBellPathByScan() {
  constexpr const char* dirs[] = {"/sdcard/images", "/sdcard/media/images"};
  std::string fallback_any_a856;
  for (const char* dir_path : dirs) {
    DIR* dir = opendir(dir_path);
    if (!dir) continue;
    struct dirent* ent = nullptr;
    while ((ent = readdir(dir)) != nullptr) {
      if (ent->d_name[0] == '.') continue;
      std::string full = std::string(dir_path) + "/" + ent->d_name;
      if (!fileExistsPosix(full.c_str())) continue;
      FILE* f = fopen(full.c_str(), "rb");
      if (!f) continue;
      uint16_t w = 0, h = 0;
      const bool ok = readBinaryHeader(f, "A856", &w, &h);
      fclose(f);
      if (!ok) continue;
      ESP_LOGI(TAG, "Found A856 candidate: %s (%ux%u)", full.c_str(), (unsigned)w, (unsigned)h);
      if (filenameContainsCaseInsensitive(ent->d_name, "door")) {
        closedir(dir);
        return full;
      }
      if (fallback_any_a856.empty()) fallback_any_a856 = full;
    }
    closedir(dir);
  }
  return fallback_any_a856;
}

void blendA856RowOntoCanvas(uint16_t* canvas, int canvas_w, int canvas_h, int dst_x, int dst_y,
                            const uint8_t* row, int row_pixels) {
  if (!canvas || !row || row_pixels <= 0) return;
  if (dst_y < 0 || dst_y >= canvas_h) return;
  for (int sx = 0; sx < row_pixels; ++sx) {
    const int dx = dst_x + sx;
    if (dx < 0 || dx >= canvas_w) continue;

    const size_t src_idx = static_cast<size_t>(sx) * 3;
    const uint8_t alpha = row[src_idx + 0];
    if (alpha == 0) continue;

    const uint16_t src565 =
        static_cast<uint16_t>(row[src_idx + 1]) | (static_cast<uint16_t>(row[src_idx + 2]) << 8);
    uint16_t& dst565 = canvas[static_cast<size_t>(dst_y) * canvas_w + dx];
    if (alpha == 255) {
      dst565 = src565;
      continue;
    }

    const uint16_t inv_alpha = static_cast<uint16_t>(255 - alpha);
    const uint16_t sr5 = (src565 >> 11) & 0x1F;
    const uint16_t sg6 = (src565 >> 5) & 0x3F;
    const uint16_t sb5 = src565 & 0x1F;

    const uint16_t dr5 = (dst565 >> 11) & 0x1F;
    const uint16_t dg6 = (dst565 >> 5) & 0x3F;
    const uint16_t db5 = dst565 & 0x1F;

    const uint16_t r5 = static_cast<uint16_t>((sr5 * alpha + dr5 * inv_alpha + 127) / 255);
    const uint16_t g6 = static_cast<uint16_t>((sg6 * alpha + dg6 * inv_alpha + 127) / 255);
    const uint16_t b5 = static_cast<uint16_t>((sb5 * alpha + db5 * inv_alpha + 127) / 255);
    dst565 = static_cast<uint16_t>((r5 << 11) | (g6 << 5) | b5);
  }
}

bool drawA856Centered(esp_lcd_panel_handle_t lcd, int screen_w, int screen_h, uint16_t* canvas,
                      size_t canvas_sz, const char* path) {
  if (!lcd || !canvas || !path) return false;
  const size_t need = static_cast<size_t>(screen_w) * screen_h * sizeof(uint16_t);
  if (canvas_sz < need) return false;

  FILE* f = fopen(path, "rb");
  if (!f) return false;

  uint16_t w = 0;
  uint16_t h = 0;
  if (!readBinaryHeader(f, "A856", &w, &h)) {
    fclose(f);
    return false;
  }

  fillBufferSolidRgb565(canvas, screen_w, screen_h, kColorBlack);

  const int dst_x = (screen_w - static_cast<int>(w)) / 2;
  const int dst_y = (screen_h - static_cast<int>(h)) / 2;
  std::vector<uint8_t> row(static_cast<size_t>(w) * 3);

  for (uint16_t sy = 0; sy < h; ++sy) {
    const size_t row_bytes = row.size();
    if (fread(row.data(), 1, row_bytes, f) != row_bytes) {
      fclose(f);
      return false;
    }
    blendA856RowOntoCanvas(canvas, screen_w, screen_h, dst_x, dst_y + sy, row.data(), w);
  }
  fclose(f);

  esp_lcd_panel_draw_bitmap(lcd, 0, 0, screen_w, screen_h, canvas);
  return true;
}

bool drawDoorBellStill(mjpeg_draw_ctx_t* ctx) {
  if (!ctx || !ctx->lcd || !ctx->still_canvas) return false;
  for (const char* path : kDoorBellCandidates) {
    if (!fileExistsPosix(path)) continue;
    if (drawA856Centered(ctx->lcd, ctx->screen_w, ctx->screen_h, ctx->still_canvas,
                         ctx->still_canvas_sz, path)) {
      ESP_LOGI(TAG, "Displayed still image: %s", path);
      return true;
    }
    ESP_LOGW(TAG, "Failed to render A856: %s", path);
  }
  const std::string scanned = findDoorBellPathByScan();
  if (!scanned.empty()) {
    if (drawA856Centered(ctx->lcd, ctx->screen_w, ctx->screen_h, ctx->still_canvas,
                         ctx->still_canvas_sz, scanned.c_str())) {
      ESP_LOGI(TAG, "Displayed still image (scanned): %s", scanned.c_str());
      return true;
    }
    ESP_LOGW(TAG, "Failed to render scanned A856: %s", scanned.c_str());
  }
  return false;
}

int chooseRandomClipIndex(size_t count, int prev_index) {
  if (count == 0) return -1;
  if (count == 1) return 0;
  int idx = static_cast<int>(esp_random() % count);
  if (idx == prev_index) idx = (idx + 1 + static_cast<int>(esp_random() % (count - 1))) % count;
  return idx;
}

void resetFpsStats(mjpeg_draw_ctx_t* ctx) {
  if (!ctx) return;
  ctx->fps_frames = 0;
  ctx->fps_start_us = 0;
  ctx->prep_us_accum = 0;
  ctx->decode_us_accum = 0;
  ctx->draw_us_accum = 0;
  ctx->total_us_accum = 0;
  ctx->src_bytes_accum = 0;
  ctx->prepped_bytes_accum = 0;
}

void playerTask(void*) {
#if CONFIG_PM_ENABLE
  esp_pm_lock_handle_t pm_lock = nullptr;
  if (esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "mjpeg", &pm_lock) == ESP_OK) {
    (void)esp_pm_lock_acquire(pm_lock);
  }
#endif

  ESP_LOGI(TAG, "Initializing board I2C/EXIO");
  I2C_Init();
  vTaskDelay(pdMS_TO_TICKS(50));
  TCA9554PWR_Init(0x00);
  Set_EXIO(EXIO_PIN8, Low);
  vTaskDelay(pdMS_TO_TICKS(10));

  touch_state_t touch = {};
  initTouchCst820(&touch);

  ESP_LOGI(TAG, "Initializing LCD");
  LCD_Init();
  if (!panel_handle) {
    ESP_LOGE(TAG, "LCD init failed (panel_handle=null)");
    vTaskDelete(nullptr);
    return;
  }
  Set_Backlight(100);
  fillScreenSolidRgb565(panel_handle, 480, 480, kColorBlue);

  ESP_LOGI(TAG, "Mounting SD card");
  if (sdcard_mount(kMountPoint) != ESP_OK) {
    fillScreenSolidRgb565(panel_handle, 480, 480, kColorRed);
    vTaskDelete(nullptr);
    return;
  }
  fillScreenSolidRgb565(panel_handle, 480, 480, kColorYellow);

  const char* clip_dir = nullptr;
  std::vector<std::string> random_clips = findRandomClipPaths(&clip_dir);
  const char* fallback_path = findFallbackVideoPath();
  if (!random_clips.empty()) {
    ESP_LOGI(TAG, "Random clip mode: found %u clip(s) in %s", static_cast<unsigned>(random_clips.size()),
             clip_dir ? clip_dir : "(unknown)");
  } else if (fallback_path) {
    ESP_LOGW(TAG, "No random clip dir found, falling back to single AVI: %s", fallback_path);
  } else {
    ESP_LOGE(TAG, "No AVI clips found (checked random clip dirs and fallback files)");
    fillScreenSolidRgb565(panel_handle, 480, 480, kColorMagenta);
    vTaskDelete(nullptr);
    return;
  }
  fillScreenSolidRgb565(panel_handle, 480, 480, kColorCyan);

  const int screen_w = 480;
  const int screen_h = 480;
  const size_t out_sz = static_cast<size_t>(screen_w) * screen_h * 2;
  uint8_t* outbuf = static_cast<uint8_t*>(
      heap_caps_malloc(out_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!outbuf) {
    ESP_LOGE(TAG, "No PSRAM for RGB565 output buffer");
    fillScreenSolidRgb565(panel_handle, 480, 480, kColorRed);
    vTaskDelete(nullptr);
    return;
  }

  s_video_events = xEventGroupCreate();
  if (!s_video_events) {
    ESP_LOGE(TAG, "Event group alloc failed");
    fillScreenSolidRgb565(panel_handle, 480, 480, kColorRed);
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
  ctx.prep_us_accum = 0;
  ctx.decode_us_accum = 0;
  ctx.draw_us_accum = 0;
  ctx.total_us_accum = 0;
  ctx.src_bytes_accum = 0;
  ctx.prepped_bytes_accum = 0;
  ctx.render_enabled = true;
  ctx.jpeg_workbuf = static_cast<uint8_t*>(
      heap_caps_malloc(kJpegWorkingBufferSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  ctx.jpeg_workbuf_sz = ctx.jpeg_workbuf ? kJpegWorkingBufferSize : 0;
  ctx.still_canvas = static_cast<uint16_t*>(
      heap_caps_malloc(out_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  ctx.still_canvas_sz = ctx.still_canvas ? out_sz : 0;

  if (ctx.jpeg_workbuf) {
    ESP_LOGI(TAG, "esp_jpeg working buffer: %u bytes (internal RAM)",
             static_cast<unsigned>(ctx.jpeg_workbuf_sz));
  } else {
    ESP_LOGW(TAG, "No internal RAM for esp_jpeg working buffer (decoder will self-allocate)");
  }
  if (!ctx.still_canvas) {
    ESP_LOGW(TAG, "No PSRAM for still_canvas; touch still-image mode will be disabled");
  }

  avi_player_config_t cfg = {
      .buffer_size = kAviPlayerBufferSize,
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

  ESP_LOGI(TAG, "Heap before avi_player_init: internal free=%u largest=%u psram free=%u",
           static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
           static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
           static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
  ESP_LOGI(TAG, "avi_player buffer_size=%u", static_cast<unsigned>(cfg.buffer_size));

  avi_player_handle_t player = nullptr;
  esp_err_t err = avi_player_init(cfg, &player);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "avi_player_init failed: %s", esp_err_to_name(err));
    fillScreenSolidRgb565(panel_handle, 480, 480, kColorRed);
    vTaskDelete(nullptr);
    return;
  }

  bool still_mode = false;
  bool clip_active = false;
  int current_clip_index = -1;
  const char* current_path = fallback_path;

  while (true) {
    if (!clip_active && !still_mode) {
      if (!random_clips.empty()) {
        current_clip_index = chooseRandomClipIndex(random_clips.size(), current_clip_index);
        current_path = (current_clip_index >= 0)
                           ? random_clips[static_cast<size_t>(current_clip_index)].c_str()
                           : nullptr;
      } else {
        current_path = fallback_path;
      }

      if (!current_path) {
        ESP_LOGE(TAG, "No playable AVI path available");
        fillScreenSolidRgb565(panel_handle, 480, 480, kColorRed);
        vTaskDelay(pdMS_TO_TICKS(1000));
      } else {
        xEventGroupClearBits(s_video_events, kVideoDoneBit);
        resetFpsStats(&ctx);
        ctx.render_enabled = true;  // keep last frame on-screen until new clip draws
        ESP_LOGI(TAG, "MJPEG AVI start: %s", current_path);
        err = avi_player_play_from_file(player, current_path);
        if (err != ESP_OK) {
          ESP_LOGE(TAG, "avi_player_play_from_file failed: %s", esp_err_to_name(err));
          fillScreenSolidRgb565(panel_handle, 480, 480, kColorRed);
          vTaskDelay(pdMS_TO_TICKS(500));
        } else {
          clip_active = true;
        }
      }
    }

    EventBits_t bits = 0;
    if (clip_active) {
      bits = xEventGroupWaitBits(s_video_events, kVideoDoneBit, pdTRUE, pdFALSE, kPollSliceTicks);
      if ((bits & kVideoDoneBit) != 0) {
        clip_active = false;
      }
    } else {
      vTaskDelay(kPollSliceTicks);
    }

    if (pollTouchToggleEvent(&touch)) {
      still_mode = !still_mode;
      ctx.render_enabled = !still_mode;
      if (still_mode) {
        // Hide ongoing AVI frames immediately; keep video parser running (1s clips) and show still.
        if (!drawDoorBellStill(&ctx)) {
          ESP_LOGW(TAG, "Door bell A856 not found/readable; using solid screen");
          fillScreenSolidRgb565(panel_handle, 480, 480, kColorMagenta);
        }
        ESP_LOGI(TAG, "Touch toggle -> still image mode");
      } else {
        ESP_LOGI(TAG, "Touch toggle -> random AVI mode");
        // If a clip is still active, frames resume; otherwise next loop starts another random clip.
      }
    }
  }
}
}  // namespace

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "magic-eye round MJPEG AVI (pure IDF) starting");
  xTaskCreatePinnedToCore(playerTask, "player_task", 8192, nullptr, 5, nullptr, 0);
}
