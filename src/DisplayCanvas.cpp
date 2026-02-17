#include "DisplayCanvas.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>

#include "Display_ST7701.h"

DisplayCanvas::~DisplayCanvas() {
  if (ownsFrames_ && frameA_) {
    heap_caps_free(frameA_);
    frameA_ = nullptr;
  }
  if (ownsFrames_ && frameB_) {
    heap_caps_free(frameB_);
    frameB_ = nullptr;
  }
  drawBuffer_ = nullptr;
  backend_ = Backend::kNone;
  ownsFrames_ = false;
}

bool DisplayCanvas::begin(bool preferPanelFrameBuffers) {
  if (preferPanelFrameBuffers && initPanelFrameBuffers()) {
    return true;
  }
  return initCustomFrameBuffers();
}

DisplayCanvas::Backend DisplayCanvas::backend() const { return backend_; }

const char* DisplayCanvas::backendName() const {
  switch (backend_) {
    case Backend::kPanelFrameBuffers:
      return "panel_frame_buffers";
    case Backend::kCustomPsrambuffers:
      return "custom_psram_buffers";
    case Backend::kNone:
    default:
      return "none";
  }
}

uint16_t* DisplayCanvas::frameBuffer() { return drawBuffer_; }

size_t DisplayCanvas::frameBytes() const { return kFrameBytes; }

bool DisplayCanvas::initPanelFrameBuffers() {
#if DISPLAY_CANVAS_ENABLE_PANEL_FB_API
  if (!panel_handle) {
    return false;
  }

  void* buf1 = nullptr;
  void* buf2 = nullptr;
  esp_err_t err = esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2);
  if (err != ESP_OK || !buf1 || !buf2) {
    return false;
  }

  frameA_ = static_cast<uint16_t*>(buf1);
  frameB_ = static_cast<uint16_t*>(buf2);
  drawBuffer_ = frameA_;
  backend_ = Backend::kPanelFrameBuffers;
  ownsFrames_ = false;
  return true;
#else
  return false;
#endif
}

bool DisplayCanvas::initCustomFrameBuffers() {
  frameA_ = static_cast<uint16_t*>(
      heap_caps_malloc(kFrameBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  frameB_ = static_cast<uint16_t*>(
      heap_caps_malloc(kFrameBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

  if (!frameA_ || !frameB_) {
    if (frameA_) {
      heap_caps_free(frameA_);
      frameA_ = nullptr;
    }
    if (frameB_) {
      heap_caps_free(frameB_);
      frameB_ = nullptr;
    }
    drawBuffer_ = nullptr;
    backend_ = Backend::kNone;
    ownsFrames_ = false;
    return false;
  }

  drawBuffer_ = frameA_;
  backend_ = Backend::kCustomPsrambuffers;
  ownsFrames_ = true;
  return true;
}

void DisplayCanvas::flipDrawBuffer() {
  drawBuffer_ = (drawBuffer_ == frameA_) ? frameB_ : frameA_;
}

void DisplayCanvas::drawHSpan(int y, int x0, int x1, uint16_t color) {
  if (y < 0 || y >= kHeight) return;

  if (x0 > x1) {
    const int t = x0;
    x0 = x1;
    x1 = t;
  }

  if (x1 < 0 || x0 >= kWidth) return;

  x0 = max(0, x0);
  x1 = min(kWidth - 1, x1);

  uint16_t* row = drawBuffer_ + (y * kWidth) + x0;
  for (int x = x0; x <= x1; ++x) {
    *row++ = color;
  }
}

void DisplayCanvas::fillScreen(uint16_t color) {
  if (!drawBuffer_) return;
  for (size_t i = 0; i < kPixelCount; ++i) {
    drawBuffer_[i] = color;
  }
}

void DisplayCanvas::fillCircle(int cx, int cy, int radius, uint16_t color) {
  if (!drawBuffer_ || radius <= 0) return;

  const int r2 = radius * radius;
  for (int dy = -radius; dy <= radius; ++dy) {
    const int y = cy + dy;
    if (y < 0 || y >= kHeight) continue;

    const int xspan = static_cast<int>(sqrtf(static_cast<float>(r2 - dy * dy)));
    drawHSpan(y, cx - xspan, cx + xspan, color);
  }
}

void DisplayCanvas::blitRgb565(const uint16_t* src, int srcWidth, int srcHeight,
                               int dstX, int dstY) {
  if (!drawBuffer_ || !src || srcWidth <= 0 || srcHeight <= 0) return;

  for (int sy = 0; sy < srcHeight; ++sy) {
    const int dy = dstY + sy;
    if (dy < 0 || dy >= kHeight) continue;

    int sxStart = 0;
    int dx = dstX;
    if (dx < 0) {
      sxStart = -dx;
      dx = 0;
    }
    if (dx >= kWidth || sxStart >= srcWidth) continue;

    int copyWidth = srcWidth - sxStart;
    if (dx + copyWidth > kWidth) {
      copyWidth = kWidth - dx;
    }
    if (copyWidth <= 0) continue;

    const uint16_t* srcRow = src + (sy * srcWidth) + sxStart;
    uint16_t* dstRow = drawBuffer_ + (dy * kWidth) + dx;
    memcpy(dstRow, srcRow, static_cast<size_t>(copyWidth) * sizeof(uint16_t));
  }
}

void DisplayCanvas::blendArgb1555(const uint16_t* src, int srcWidth,
                                  int srcHeight, int dstX, int dstY) {
  if (!drawBuffer_ || !src || srcWidth <= 0 || srcHeight <= 0) return;

  for (int sy = 0; sy < srcHeight; ++sy) {
    const int dy = dstY + sy;
    if (dy < 0 || dy >= kHeight) continue;
    for (int sx = 0; sx < srcWidth; ++sx) {
      const int dx = dstX + sx;
      if (dx < 0 || dx >= kWidth) continue;

      const uint16_t px = src[sy * srcWidth + sx];
      if ((px & 0x8000) == 0) continue;
      drawBuffer_[dy * kWidth + dx] = px & 0x7FFF;
    }
  }
}

void DisplayCanvas::blendA856(const uint8_t* src, int srcWidth, int srcHeight,
                              int dstX, int dstY) {
  if (!drawBuffer_ || !src || srcWidth <= 0 || srcHeight <= 0) return;

  for (int sy = 0; sy < srcHeight; ++sy) {
    const int dy = dstY + sy;
    if (dy < 0 || dy >= kHeight) continue;

    for (int sx = 0; sx < srcWidth; ++sx) {
      const int dx = dstX + sx;
      if (dx < 0 || dx >= kWidth) continue;

      const size_t srcIdx = static_cast<size_t>(sy * srcWidth + sx) * 3;
      const uint8_t alpha = src[srcIdx + 0];
      if (alpha == 0) continue;

      const uint16_t src565 =
          static_cast<uint16_t>(src[srcIdx + 1]) |
          (static_cast<uint16_t>(src[srcIdx + 2]) << 8);

      if (alpha == 255) {
        drawBuffer_[dy * kWidth + dx] = src565;
        continue;
      }

      const uint16_t dst565 = drawBuffer_[dy * kWidth + dx];
      const uint16_t invAlpha = static_cast<uint16_t>(255 - alpha);

      const uint16_t sr5 = (src565 >> 11) & 0x1F;
      const uint16_t sg6 = (src565 >> 5) & 0x3F;
      const uint16_t sb5 = src565 & 0x1F;

      const uint16_t dr5 = (dst565 >> 11) & 0x1F;
      const uint16_t dg6 = (dst565 >> 5) & 0x3F;
      const uint16_t db5 = dst565 & 0x1F;

      const uint16_t r5 =
          static_cast<uint16_t>((sr5 * alpha + dr5 * invAlpha + 127) / 255);
      const uint16_t g6 =
          static_cast<uint16_t>((sg6 * alpha + dg6 * invAlpha + 127) / 255);
      const uint16_t b5 =
          static_cast<uint16_t>((sb5 * alpha + db5 * invAlpha + 127) / 255);

      drawBuffer_[dy * kWidth + dx] = (r5 << 11) | (g6 << 5) | b5;
    }
  }
}

void DisplayCanvas::present() {
  if (!drawBuffer_) return;

  esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, kWidth, kHeight, drawBuffer_);
  flipDrawBuffer();
}
