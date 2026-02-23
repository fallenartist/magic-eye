#include "MediaDemo.h"

#include <esp_heap_caps.h>
#include <string.h>

#include "TCA9554PWR.h"

namespace {
constexpr int kSdClkPin = 2;
constexpr int kSdCmdPin = 1;
constexpr int kSdD0Pin = 42;
constexpr uint16_t kColorBlack = 0x0000;
constexpr size_t kVideoPreloadChunkBytes = 4096;
constexpr uint8_t kVideoPreloadYieldEveryChunks = 2;
constexpr uint8_t kVideoPreloadYieldMs = 1;
}  // namespace

MediaDemo::~MediaDemo() {
  freeVideoGroupCache();
  if (scratch_) {
    heap_caps_free(scratch_);
    scratch_ = nullptr;
    scratchBytes_ = 0;
  }
}

bool MediaDemo::begin(DisplayCanvas* canvas) {
  canvas_ = canvas;
  sdReady_ = initSdCard();
  mode_ = Mode::kVideo;
  modeDirty_ = true;
  sequenceIndex_ = 0;
  videoIndex_ = 0;
  sequenceWarned_ = false;
  videoWarned_ = false;
  videoFrameCount_ = 0;
  videoFrameCountScanned_ = false;
  videoGroupStartIndex_ = 0;
  videoGroupOffset_ = 0;
  videoGroupCachedFrames_ = 0;
  videoNextFrameDueMs_ = 0;
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

void MediaDemo::showDoorBell() {
  mode_ = Mode::kRoundImage;
  modeDirty_ = true;
  resetCounters(millis());
  Serial.println("Mode: door_bell");
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

void MediaDemo::freeVideoGroupCache() {
  if (videoGroupCache_) {
    heap_caps_free(videoGroupCache_);
    videoGroupCache_ = nullptr;
  }
  videoGroupCacheBytes_ = 0;
  videoGroupCachedFrames_ = 0;
}

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
        const bool bellOk =
            loadAlphaStill(kPathDoorBellA856, kPathDoorBellA155, -1, -1, true, true);
        const bool roundOk = bellOk ? true
                                    : loadAlphaStill(kPathRoundA856, kPathRoundA155, -1,
                                                     -1, true, true);
        if (roundOk) {
          Serial.printf("Rendered round image (%s preferred)\n",
                        bellOk ? kPathDoorBellA856 : kPathRoundA856);
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
        videoGroupOffset_ = 0;
        videoGroupCachedFrames_ = 0;
        videoNextFrameDueMs_ = 0;
        if (!videoFrameCountScanned_) {
          scanVideoFrameCount();
        }
        chooseRandomVideoGroup();
        preloadRandomVideoGroup();
        nowMs = millis();
        modeDirty_ = false;
      }
      if (presentCachedVideoGroupFrame(nowMs)) {
        break;
      }
      if (!loadRandomVideoGroupFrame(nowMs)) {
        loadNextLoopedFrame(kDirVideo, videoIndex_, videoWarned_, "video", nowMs);
      }
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

bool MediaDemo::ensureVideoGroupCache(uint16_t frameCount) {
  const size_t frameBytes =
      static_cast<size_t>(DisplayCanvas::kWidth) * DisplayCanvas::kHeight *
      sizeof(uint16_t);
  const size_t bytes = frameBytes * static_cast<size_t>(frameCount);
  if (videoGroupCache_ && videoGroupCacheBytes_ >= bytes) {
    return true;
  }

  freeVideoGroupCache();
  videoGroupCache_ = static_cast<uint8_t*>(
      heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!videoGroupCache_) {
    return false;
  }
  videoGroupCacheBytes_ = bytes;
  return true;
}

bool MediaDemo::readVideoFrameToBuffer(const char* path, uint8_t* dst, size_t bytes) {
  static uint8_t chunkBuf[kVideoPreloadChunkBytes];

  File f = SD_MMC.open(path, FILE_READ);
  if (!f) {
    return false;
  }

  uint16_t w = 0;
  uint16_t h = 0;
  if (!readHeader(f, "R565", w, h) || w != DisplayCanvas::kWidth ||
      h != DisplayCanvas::kHeight) {
    f.close();
    return false;
  }

  size_t offset = 0;
  uint32_t chunkCount = 0;
  while (offset < bytes) {
    const size_t remaining = bytes - offset;
    const size_t toRead =
        (remaining > sizeof(chunkBuf)) ? sizeof(chunkBuf) : remaining;
    const int n = f.read(chunkBuf, toRead);
    if (n != static_cast<int>(toRead)) {
      f.close();
      return false;
    }

    memcpy(dst + offset, chunkBuf, toRead);
    offset += toRead;

    ++chunkCount;
    if ((chunkCount % kVideoPreloadYieldEveryChunks) == 0) {
      delay(kVideoPreloadYieldMs);
    } else {
      yield();
    }
  }
  f.close();
  return true;
}

bool MediaDemo::scanVideoFrameCount() {
  videoFrameCountScanned_ = true;
  videoFrameCount_ = 0;

  char path[96] = {0};
  for (uint32_t i = 0; i < 10000; ++i) {
    snprintf(path, sizeof(path), "%s/%04lu.r565", kDirVideo,
             static_cast<unsigned long>(i));
    File f = SD_MMC.open(path, FILE_READ);
    if (!f) {
      break;
    }
    f.close();
    videoFrameCount_ = i + 1;
  }

  Serial.printf("video groups: detected %lu frame(s) in %s\n",
                static_cast<unsigned long>(videoFrameCount_), kDirVideo);
  return videoFrameCount_ > 0;
}

void MediaDemo::chooseRandomVideoGroup() {
  videoGroupOffset_ = 0;
  if (videoFrameCount_ < kVideoGroupFrames) {
    videoGroupStartIndex_ = 0;
    return;
  }

  const uint32_t groupCount = videoFrameCount_ / kVideoGroupFrames;
  const uint32_t group = static_cast<uint32_t>(random(static_cast<long>(groupCount)));
  videoGroupStartIndex_ = group * kVideoGroupFrames;
  videoIndex_ = videoGroupStartIndex_;
  Serial.printf("video groups: selected group %lu/%lu (frames %lu-%lu)\n",
                static_cast<unsigned long>(group + 1),
                static_cast<unsigned long>(groupCount),
                static_cast<unsigned long>(videoGroupStartIndex_),
                static_cast<unsigned long>(videoGroupStartIndex_ + kVideoGroupFrames - 1));
}

bool MediaDemo::preloadRandomVideoGroup() {
  if (!canvas_ || videoFrameCount_ == 0) {
    return false;
  }

  uint16_t framesToTry = kVideoGroupFrames;
  if (videoFrameCount_ < framesToTry) {
    framesToTry = static_cast<uint16_t>(videoFrameCount_);
  }

  while (framesToTry >= kVideoGroupMinFrames && !ensureVideoGroupCache(framesToTry)) {
    --framesToTry;
  }
  if (framesToTry < kVideoGroupMinFrames) {
    Serial.println("video cache: alloc failed, using SD streaming");
    freeVideoGroupCache();
    return false;
  }

  const size_t frameBytes = canvas_->frameBytes();
  const uint32_t loadStartMs = millis();
  char path[96] = {0};
  uint16_t loaded = 0;
  for (; loaded < framesToTry; ++loaded) {
    const uint32_t frame = videoGroupStartIndex_ + loaded;
    snprintf(path, sizeof(path), "%s/%04lu.r565", kDirVideo,
             static_cast<unsigned long>(frame));
    if (!readVideoFrameToBuffer(path, videoGroupCache_ + (loaded * frameBytes),
                                frameBytes)) {
      break;
    }
  }

  if (loaded == 0) {
    videoGroupCachedFrames_ = 0;
    videoGroupOffset_ = 0;
    return false;
  }

  videoGroupCachedFrames_ = loaded;
  videoGroupOffset_ = 0;
  videoNextFrameDueMs_ = 0;
  // Measure burst playback FPS separately from SD preload pauses.
  resetCounters(millis());
  Serial.printf("video cache: loaded %u frame(s) in %lums (start=%lu)\n", loaded,
                static_cast<unsigned long>(millis() - loadStartMs),
                static_cast<unsigned long>(videoGroupStartIndex_));
  return true;
}

bool MediaDemo::presentCachedVideoGroupFrame(uint32_t nowMs) {
  if (!canvas_ || !videoGroupCache_ || videoGroupCachedFrames_ == 0) {
    return false;
  }

  if (videoNextFrameDueMs_ != 0 && nowMs < videoNextFrameDueMs_) {
    return true;
  }

  const size_t frameBytes = canvas_->frameBytes();
  const uint8_t* framePtr =
      videoGroupCache_ + (static_cast<size_t>(videoGroupOffset_) * frameBytes);

  if (canvas_->frameBuffer() && canvas_->frameBytes() == frameBytes) {
    memcpy(canvas_->frameBuffer(), framePtr, frameBytes);
  } else {
    canvas_->blitRgb565(reinterpret_cast<const uint16_t*>(framePtr),
                       DisplayCanvas::kWidth, DisplayCanvas::kHeight, 0, 0);
  }
  canvas_->present();

  ++videoGroupOffset_;
  ++fpsFrames_;
  logFps("video_buf", nowMs);

  const uint32_t frameIntervalMs = 1000UL / kVideoTargetFps;
  videoNextFrameDueMs_ = (videoNextFrameDueMs_ == 0)
                             ? (nowMs + frameIntervalMs)
                             : (videoNextFrameDueMs_ + frameIntervalMs);
  if (videoNextFrameDueMs_ < nowMs) {
    videoNextFrameDueMs_ = nowMs + frameIntervalMs;
  }

  if (videoGroupOffset_ >= videoGroupCachedFrames_) {
    chooseRandomVideoGroup();
    if (!preloadRandomVideoGroup()) {
      videoGroupCachedFrames_ = 0;
      return false;
    }
  }
  return true;
}

bool MediaDemo::loadRandomVideoGroupFrame(uint32_t nowMs) {
  if (videoFrameCount_ < kVideoGroupFrames) {
    return false;
  }

  const uint32_t frame = videoGroupStartIndex_ + videoGroupOffset_;
  char path[96] = {0};
  snprintf(path, sizeof(path), "%s/%04lu.r565", kDirVideo,
           static_cast<unsigned long>(frame));
  if (!loadR565(path, false, true)) {
    if (!videoWarned_) {
      videoWarned_ = true;
      Serial.printf("video groups: missing frame %s\n", path);
    }
    return false;
  }

  videoWarned_ = false;
  ++videoGroupOffset_;
  if (videoGroupOffset_ >= kVideoGroupFrames) {
    chooseRandomVideoGroup();
  }

  ++fpsFrames_;
  logFps("video_seq", nowMs);
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
