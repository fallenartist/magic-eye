#pragma once

#include <stddef.h>
#include <stdint.h>

#ifndef DISPLAY_CANVAS_ENABLE_PANEL_FB_API
#define DISPLAY_CANVAS_ENABLE_PANEL_FB_API 0
#endif

#ifndef DISPLAY_CANVAS_PANEL_FB_COUNT
#define DISPLAY_CANVAS_PANEL_FB_COUNT 2
#endif

class DisplayCanvas {
 public:
  enum class Backend {
    kNone,
    kPanelFrameBuffers,
    kCustomPsrambuffers,
  };

  static constexpr int kWidth = 480;
  static constexpr int kHeight = 480;

  DisplayCanvas() = default;
  ~DisplayCanvas();

  bool begin(bool preferPanelFrameBuffers = true);
  Backend backend() const;
  const char* backendName() const;
  uint16_t* frameBuffer();
  size_t frameBytes() const;

  void fillScreen(uint16_t color);
  void fillCircle(int cx, int cy, int radius, uint16_t color);
  void blitRgb565(const uint16_t* src, int srcWidth, int srcHeight, int dstX,
                  int dstY);
  void blendArgb1555(const uint16_t* src, int srcWidth, int srcHeight, int dstX,
                     int dstY);
  void blendA856(const uint8_t* src, int srcWidth, int srcHeight, int dstX,
                 int dstY);
  void present();

 private:
  static constexpr size_t kMaxFrameBuffers = 3;
  static constexpr size_t kPixelCount = static_cast<size_t>(kWidth) * kHeight;
  static constexpr size_t kFrameBytes = kPixelCount * sizeof(uint16_t);

  uint16_t* frames_[kMaxFrameBuffers] = {nullptr, nullptr, nullptr};
  uint8_t frameCount_ = 0;
  uint8_t drawIndex_ = 0;
  Backend backend_ = Backend::kNone;
  bool ownsFrames_ = false;

  bool initPanelFrameBuffers();
  bool initCustomFrameBuffers();

  void flipDrawBuffer();
  void drawHSpan(int y, int x0, int x1, uint16_t color);
};
