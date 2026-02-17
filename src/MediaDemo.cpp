#include "MediaDemo.h"

#include <esp_heap_caps.h>
#include <string.h>

#include "TCA9554PWR.h"

namespace {
constexpr int kSdClkPin = 2;
constexpr int kSdCmdPin = 1;
constexpr int kSdD0Pin = 42;
constexpr uint16_t kColorBlack = 0x0000;
}  // namespace

MediaDemo::~MediaDemo() {
  if (scratch_) {
    heap_caps_free(scratch_);
    scratch_ = nullptr;
    scratchBytes_ = 0;
  }
}

bool MediaDemo::begin(DisplayCanvas* canvas) {
  canvas_ = canvas;
  sdReady_ = initSdCard();
  mode_ = Mode::kFullImage;
  modeDirty_ = true;
  sequenceIndex_ = 0;
  videoIndex_ = 0;
  sequenceWarned_ = false;
  videoWarned_ = false;
  resetCounters(millis());
  return sdReady_;
}

void MediaDemo::advanceMode() {
  const uint8_t next = (static_cast<uint8_t>(mode_) + 1) %
                       static_cast<uint8_t>(Mode::kCount);
  mode_ = static_cast<Mode>(next);
  modeDirty_ = true;
  resetCounters(millis());
  Serial.printf("Mode: %s\n", modeName());
}

const char* MediaDemo::modeName() const {
  switch (mode_) {
    case Mode::kFullImage:
      return "full_image";
    case Mode::kImageSequence:
      return "image_sequence";
    case Mode::kRoundImage:
      return "round_image";
    case Mode::kAlphaOverlay:
      return "alpha_overlay";
    case Mode::kVideo:
      return "video";
    case Mode::kCount:
    default:
      return "unknown";
  }
}

bool MediaDemo::sdReady() const { return sdReady_; }

void MediaDemo::update(uint32_t nowMs) {
  if (!canvas_) return;
  if (!sdReady_) {
    if (modeDirty_) {
      canvas_->fillScreen(kColorBlack);
      canvas_->present();
      modeDirty_ = false;
    }
    return;
  }

  switch (mode_) {
    case Mode::kFullImage:
      if (modeDirty_) {
        if (loadR565(kPathFull, true, true)) {
          Serial.printf("Rendered %s\n", kPathFull);
        }
        modeDirty_ = false;
      }
      break;

    case Mode::kImageSequence:
      if (modeDirty_) {
        sequenceIndex_ = 0;
        sequenceWarned_ = false;
        modeDirty_ = false;
      }
      loadNextLoopedFrame(kDirSequence, sequenceIndex_, sequenceWarned_,
                          "sequence", nowMs);
      break;

    case Mode::kRoundImage:
      if (modeDirty_) {
        if (loadAlphaStill(kPathRoundA856, kPathRoundA155, -1, -1, true, true)) {
          Serial.printf("Rendered round image (%s preferred)\n", kPathRoundA856);
        }
        modeDirty_ = false;
      }
      break;

    case Mode::kAlphaOverlay:
      if (modeDirty_) {
        const bool baseOk = loadR565(kPathBase, true, false);
        const bool overlayOk = loadAlphaStill(
            kPathOverlayA856, kPathOverlayA155, 0, 0, !baseOk, false);
        if (baseOk || overlayOk) {
          canvas_->present();
          Serial.printf("Rendered alpha overlay (%s + %s preferred)\n", kPathBase,
                        kPathOverlayA856);
        }
        modeDirty_ = false;
      }
      break;

    case Mode::kVideo:
      if (modeDirty_) {
        videoIndex_ = 0;
        videoWarned_ = false;
        modeDirty_ = false;
      }
      loadNextLoopedFrame(kDirVideo, videoIndex_, videoWarned_, "video", nowMs);
      break;

    case Mode::kCount:
      break;
  }
}

bool MediaDemo::initSdCard() {
  if (!SD_MMC.setPins(kSdClkPin, kSdCmdPin, kSdD0Pin, -1, -1, -1)) {
    Serial.println("SD: setPins failed");
    return false;
  }

  // Board routes SD D3 through the external IO expander.
  Set_EXIO(EXIO_PIN4, High);
  delay(10);

  if (!SD_MMC.begin("/sdcard", true, true)) {
    Serial.println("SD: begin failed");
    return false;
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("SD: no card");
    return false;
  }

  Serial.printf("SD: total=%llu used=%llu\n", SD_MMC.totalBytes(),
                SD_MMC.usedBytes());
  return true;
}

bool MediaDemo::ensureScratch(size_t bytes) {
  if (scratch_ && scratchBytes_ >= bytes) {
    return true;
  }
  if (scratch_) {
    heap_caps_free(scratch_);
    scratch_ = nullptr;
    scratchBytes_ = 0;
  }

  scratch_ = static_cast<uint8_t*>(
      heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!scratch_) {
    Serial.println("Scratch alloc failed");
    return false;
  }

  scratchBytes_ = bytes;
  return true;
}

bool MediaDemo::readHeader(File& f, const char expectedMagic[4], uint16_t& w,
                           uint16_t& h) {
  uint8_t header[8] = {0};
  if (f.read(header, sizeof(header)) != sizeof(header)) {
    return false;
  }
  if (memcmp(header, expectedMagic, 4) != 0) {
    return false;
  }

  w = static_cast<uint16_t>(header[4] | (header[5] << 8));
  h = static_cast<uint16_t>(header[6] | (header[7] << 8));
  if (w == 0 || h == 0) {
    return false;
  }
  return true;
}

bool MediaDemo::loadR565(const char* path, bool clearBackground, bool present) {
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) {
    Serial.printf("Missing file: %s\n", path);
    return false;
  }

  uint16_t w = 0;
  uint16_t h = 0;
  if (!readHeader(f, "R565", w, h)) {
    Serial.printf("Invalid R565 header: %s\n", path);
    f.close();
    return false;
  }

  const size_t pixels = static_cast<size_t>(w) * h;
  const size_t bytes = pixels * sizeof(uint16_t);

  if (w == DisplayCanvas::kWidth && h == DisplayCanvas::kHeight &&
      canvas_->frameBuffer() && canvas_->frameBytes() == bytes) {
    if (f.read(reinterpret_cast<uint8_t*>(canvas_->frameBuffer()), bytes) !=
        static_cast<int>(bytes)) {
      Serial.printf("Short read: %s\n", path);
      f.close();
      return false;
    }
  } else {
    if (!ensureScratch(bytes)) {
      f.close();
      return false;
    }
    if (f.read(scratch_, bytes) != static_cast<int>(bytes)) {
      Serial.printf("Short read: %s\n", path);
      f.close();
      return false;
    }
    if (clearBackground) {
      canvas_->fillScreen(kColorBlack);
    }
    canvas_->blitRgb565(reinterpret_cast<const uint16_t*>(scratch_), w, h,
                        (DisplayCanvas::kWidth - w) / 2,
                        (DisplayCanvas::kHeight - h) / 2);
  }

  f.close();
  if (present) {
    canvas_->present();
  }
  return true;
}

bool MediaDemo::loadA155(const char* path, int dstX, int dstY, bool clearBackground,
                         bool present) {
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) {
    Serial.printf("Missing file: %s\n", path);
    return false;
  }

  uint16_t w = 0;
  uint16_t h = 0;
  if (!readHeader(f, "A155", w, h)) {
    Serial.printf("Invalid A155 header: %s\n", path);
    f.close();
    return false;
  }

  const size_t pixels = static_cast<size_t>(w) * h;
  const size_t bytes = pixels * sizeof(uint16_t);
  if (!ensureScratch(bytes)) {
    f.close();
    return false;
  }

  if (f.read(scratch_, bytes) != static_cast<int>(bytes)) {
    Serial.printf("Short read: %s\n", path);
    f.close();
    return false;
  }
  f.close();

  if (clearBackground) {
    canvas_->fillScreen(kColorBlack);
  }
  if (dstX < 0) {
    dstX = (DisplayCanvas::kWidth - w) / 2;
  }
  if (dstY < 0) {
    dstY = (DisplayCanvas::kHeight - h) / 2;
  }
  canvas_->blendArgb1555(reinterpret_cast<const uint16_t*>(scratch_), w, h,
                         dstX, dstY);
  if (present) {
    canvas_->present();
  }
  return true;
}

bool MediaDemo::loadA856(const char* path, int dstX, int dstY,
                         bool clearBackground, bool present) {
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) {
    Serial.printf("Missing file: %s\n", path);
    return false;
  }

  uint16_t w = 0;
  uint16_t h = 0;
  if (!readHeader(f, "A856", w, h)) {
    Serial.printf("Invalid A856 header: %s\n", path);
    f.close();
    return false;
  }

  const size_t pixels = static_cast<size_t>(w) * h;
  const size_t bytes = pixels * 3;
  if (!ensureScratch(bytes)) {
    f.close();
    return false;
  }

  if (f.read(scratch_, bytes) != static_cast<int>(bytes)) {
    Serial.printf("Short read: %s\n", path);
    f.close();
    return false;
  }
  f.close();

  if (clearBackground) {
    canvas_->fillScreen(kColorBlack);
  }
  if (dstX < 0) {
    dstX = (DisplayCanvas::kWidth - w) / 2;
  }
  if (dstY < 0) {
    dstY = (DisplayCanvas::kHeight - h) / 2;
  }
  canvas_->blendA856(scratch_, w, h, dstX, dstY);
  if (present) {
    canvas_->present();
  }
  return true;
}

bool MediaDemo::loadAlphaStill(const char* pathA856, const char* pathA155,
                               int dstX, int dstY, bool clearBackground,
                               bool present) {
  if (loadA856(pathA856, dstX, dstY, clearBackground, present)) {
    return true;
  }
  return loadA155(pathA155, dstX, dstY, clearBackground, present);
}

bool MediaDemo::loadNextLoopedFrame(const char* directory, uint32_t& frameIndex,
                                    bool& warnedNoFrames, const char* tag,
                                    uint32_t nowMs) {
  char path[96] = {0};
  snprintf(path, sizeof(path), "%s/%04lu.r565", directory,
           static_cast<unsigned long>(frameIndex));

  if (!loadR565(path, false, true)) {
    if (frameIndex > 0) {
      frameIndex = 0;
      snprintf(path, sizeof(path), "%s/%04lu.r565", directory,
               static_cast<unsigned long>(frameIndex));
      if (!loadR565(path, false, true)) {
        if (!warnedNoFrames) {
          warnedNoFrames = true;
          Serial.printf("%s: no readable frames in %s\n", tag, directory);
        }
        return false;
      }
    } else {
      if (!warnedNoFrames) {
        warnedNoFrames = true;
        Serial.printf("%s: no readable frames in %s\n", tag, directory);
      }
      return false;
    }
  }

  warnedNoFrames = false;
  ++frameIndex;
  ++fpsFrames_;
  logFps(tag, nowMs);
  return true;
}

void MediaDemo::resetCounters(uint32_t nowMs) {
  fpsStartMs_ = nowMs;
  fpsFrames_ = 0;
}

void MediaDemo::logFps(const char* tag, uint32_t nowMs) {
  const uint32_t elapsed = nowMs - fpsStartMs_;
  if (elapsed < 1000) {
    return;
  }
  const float fps = (1000.0f * static_cast<float>(fpsFrames_)) /
                    static_cast<float>(elapsed);
  Serial.printf("%s fps: %.2f\n", tag, fps);
  resetCounters(nowMs);
}
