#include "MediaDemo.h"

#include <esp_heap_caps.h>
#include <esp_rom_tjpgd.h>
#include <string.h>

#include "Display_ST7701.h"
#include "TCA9554PWR.h"

namespace {
constexpr int kSdClkPin = 2;
constexpr int kSdCmdPin = 1;
constexpr int kSdD0Pin = 42;
constexpr bool kSd1BitMode = true;
constexpr uint32_t kSdFreqKHz = SDMMC_FREQ_HIGHSPEED;
constexpr uint8_t kSdMaxOpenFiles = 8;
constexpr size_t kVideoReadStageChunkBytes = 32 * 1024;
constexpr size_t kDt16HeaderBytes = 16;
constexpr uint16_t kDt16DefaultTileSize = 16;
constexpr size_t kDt16PsrampreloadMaxBytes = 8 * 1024 * 1024;
constexpr size_t kMjpgHeaderBytes = 16;
constexpr size_t kJpegDecodeWorkBytes = 8 * 1024;
constexpr uint16_t kColorBlack = 0x0000;
constexpr size_t kVideoFrameBytes =
    static_cast<size_t>(DisplayCanvas::kWidth) * DisplayCanvas::kHeight *
    sizeof(uint16_t);

struct JpegMemReader {
  const uint8_t* data = nullptr;
  size_t len = 0;
  size_t pos = 0;
};

struct JpegDecodeTarget {
  uint16_t* dst = nullptr;
  int dstWidth = DisplayCanvas::kWidth;
  int dstHeight = DisplayCanvas::kHeight;
  int dstX = 0;
  int dstY = 0;
};

struct JpegDecodeSession {
  JpegMemReader reader;
  JpegDecodeTarget target;
};

uint32_t tjpgdInputMem(esp_rom_tjpgd_dec_t* dec, uint8_t* buffer, uint32_t ndata) {
  if (!dec || !dec->device) return 0;
  auto* session = static_cast<JpegDecodeSession*>(dec->device);
  auto* src = &session->reader;
  if (src->pos >= src->len) {
    return 0;
  }
  const size_t avail = src->len - src->pos;
  const size_t count = min(static_cast<size_t>(ndata), avail);
  if (buffer) {
    memcpy(buffer, src->data + src->pos, count);
  }
  src->pos += count;
  return static_cast<uint32_t>(count);
}

uint32_t tjpgdOutputRgb888ToRgb565(esp_rom_tjpgd_dec_t* dec, void* bitmap,
                                   esp_rom_tjpgd_rect_t* rect) {
  if (!dec || !dec->device || !bitmap || !rect) return 0;
  auto* session = static_cast<JpegDecodeSession*>(dec->device);
  auto* target = &session->target;
  if (!target->dst) return 0;

  const uint8_t* src = static_cast<const uint8_t*>(bitmap);
  const int tileW = static_cast<int>(rect->right - rect->left + 1);
  const int tileH = static_cast<int>(rect->bottom - rect->top + 1);
  if (tileW <= 0 || tileH <= 0) return 1;

  for (int y = 0; y < tileH; ++y) {
    const int dy = target->dstY + static_cast<int>(rect->top) + y;
    if (dy < 0 || dy >= target->dstHeight) {
      src += static_cast<size_t>(tileW) * 3;
      continue;
    }
    for (int x = 0; x < tileW; ++x) {
      const int dx = target->dstX + static_cast<int>(rect->left) + x;
      const uint8_t r = *src++;
      const uint8_t g = *src++;
      const uint8_t b = *src++;
      if (dx < 0 || dx >= target->dstWidth) {
        continue;
      }
      target->dst[dy * target->dstWidth + dx] =
          static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
    }
  }
  return 1;
}
}  // namespace

const char* const MediaDemo::kStillSequence[] = {
    "/media/images/eye-1.r565",
    "/media/images/eye-2.r565",
    "/media/images/eye-3.r565",
    "/media/images/eye-4.r565",
};

MediaDemo::~MediaDemo() {
  stopVideoPrefetch();
  if (videoTaskHandle_) {
    vTaskDelete(videoTaskHandle_);
    videoTaskHandle_ = nullptr;
  }
  if (videoQueueMutex_) {
    vSemaphoreDelete(videoQueueMutex_);
    videoQueueMutex_ = nullptr;
  }
  closeDt16Stream();
  closeMjpgStream();
  closeVideoStream();

  if (scratch_) {
    heap_caps_free(scratch_);
    scratch_ = nullptr;
    scratchBytes_ = 0;
  }
  if (videoReadStageBuf_) {
    heap_caps_free(videoReadStageBuf_);
    videoReadStageBuf_ = nullptr;
    videoReadStageBufBytes_ = 0;
  }
  if (videoCompressedBuf_) {
    heap_caps_free(videoCompressedBuf_);
    videoCompressedBuf_ = nullptr;
    videoCompressedBufBytes_ = 0;
  }
  for (size_t i = 0; i < kStillCacheDepth; ++i) {
    if (stillCache_[i]) {
      heap_caps_free(stillCache_[i]);
      stillCache_[i] = nullptr;
    }
  }
  stillCacheReady_ = false;
  for (size_t i = 0; i < kVideoQueueDepth; ++i) {
    if (videoQueue_[i]) {
      heap_caps_free(videoQueue_[i]);
      videoQueue_[i] = nullptr;
    }
  }
  videoQueueReady_ = false;
}

bool MediaDemo::begin(DisplayCanvas* canvas) {
  canvas_ = canvas;
  sdReady_ = initSdCard();
  mode_ = Mode::kFullImage;
  modeDirty_ = true;
  sequenceIndex_ = 0;
  videoIndex_ = 0;
  videoLoadIndex_ = 0;
  sequenceWarned_ = false;
  videoWarned_ = false;
  videoUseDt16Stream_ = false;
  videoUseMjpgStream_ = false;
  videoUseStream_ = false;
  videoUseJpegSequence_ = false;
  videoFrameIntervalMs_ = kVideoFrameIntervalDefaultMs;
  resetVideoQueue();
  resetVideoDiagnostics(millis());
  resetCounters(millis());
  return sdReady_;
}

void MediaDemo::advanceMode() {
  if (mode_ == Mode::kVideo) {
    stopVideoPrefetch();
  }
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
      if (nowMs >= nextFrameDueMs_) {
        if (loadStillSequenceFrame(nowMs)) {
          nextFrameDueMs_ = nowMs + kSequenceFrameIntervalMs;
        } else {
          nextFrameDueMs_ = nowMs + kRetryDelayMs;
        }
      }
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
        videoLoadIndex_ = 0;
        videoWarned_ = false;
        videoFrameIntervalMs_ = kVideoFrameIntervalDefaultMs;
        resetVideoQueue();
        resetVideoDiagnostics(nowMs);
        startVideoPrefetch();
        modeDirty_ = false;
      }
      if (nowMs >= nextFrameDueMs_) {
        if (presentVideoFrameFromQueue(nowMs)) {
          nextFrameDueMs_ = nowMs + videoFrameIntervalMs_;
        } else {
          nextFrameDueMs_ = nowMs + 4;
        }
      }
      logVideoDiagnostics(nowMs);
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

  if (!SD_MMC.begin("/sdcard", kSd1BitMode, false, kSdFreqKHz, kSdMaxOpenFiles)) {
    Serial.println("SD: begin failed");
    return false;
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("SD: no card");
    return false;
  }

  Serial.printf("SD: total=%llu used=%llu mode=%s freq=%luKHz\n",
                SD_MMC.totalBytes(), SD_MMC.usedBytes(),
                kSd1BitMode ? "1-bit" : "4-bit",
                static_cast<unsigned long>(kSdFreqKHz));
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

bool MediaDemo::initStillCache() {
  if (stillCacheReady_) {
    return true;
  }

  for (size_t i = 0; i < kStillCacheDepth; ++i) {
    stillCache_[i] = static_cast<uint16_t*>(
        heap_caps_malloc(kVideoFrameBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!stillCache_[i]) {
      Serial.println("Still cache alloc failed");
      for (size_t j = 0; j <= i; ++j) {
        if (stillCache_[j]) {
          heap_caps_free(stillCache_[j]);
          stillCache_[j] = nullptr;
        }
      }
      stillCacheReady_ = false;
      return false;
    }
  }

  for (size_t i = 0; i < kStillCacheDepth; ++i) {
    if (!readR565Frame(kStillSequence[i], stillCache_[i], true)) {
      Serial.printf("Still cache load failed: %s\n", kStillSequence[i]);
      for (size_t j = 0; j < kStillCacheDepth; ++j) {
        if (stillCache_[j]) {
          heap_caps_free(stillCache_[j]);
          stillCache_[j] = nullptr;
        }
      }
      stillCacheReady_ = false;
      return false;
    }
  }

  stillCacheReady_ = true;
  return true;
}

void MediaDemo::releaseStillCache() {
  for (size_t i = 0; i < kStillCacheDepth; ++i) {
    if (stillCache_[i]) {
      heap_caps_free(stillCache_[i]);
      stillCache_[i] = nullptr;
    }
  }
  stillCacheReady_ = false;
}

bool MediaDemo::initVideoQueue() {
  if (videoQueueReady_) {
    return true;
  }
  if (!videoQueueMutex_) {
    videoQueueMutex_ = xSemaphoreCreateMutex();
    if (!videoQueueMutex_) {
      Serial.println("Video queue mutex alloc failed");
      return false;
    }
  }

  for (size_t i = 0; i < kVideoQueueDepth; ++i) {
    videoQueue_[i] = static_cast<uint16_t*>(
        heap_caps_malloc(kVideoFrameBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!videoQueue_[i]) {
      Serial.println("Video queue alloc failed");
      for (size_t j = 0; j <= i; ++j) {
        if (videoQueue_[j]) {
          heap_caps_free(videoQueue_[j]);
          videoQueue_[j] = nullptr;
        }
      }
      videoQueueReady_ = false;
      return false;
    }
  }

  videoQueueReady_ = true;
  return true;
}

void MediaDemo::resetVideoQueue() {
  if (videoQueueMutex_) {
    if (xSemaphoreTake(videoQueueMutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
      return;
    }
  }
  videoQueueCount_ = 0;
  videoQueueReadIdx_ = 0;
  videoQueueWriteIdx_ = 0;
  if (videoQueueMutex_) {
    xSemaphoreGive(videoQueueMutex_);
  }
  sampleVideoQueueDepth(0);
}

bool MediaDemo::ensureVideoTask() {
  if (videoTaskHandle_) {
    return true;
  }
  BaseType_t created = xTaskCreatePinnedToCore(
      videoTaskThunk, "video_prefetch", 6144, this, 1, &videoTaskHandle_, 0);
  if (created != pdPASS) {
    videoTaskHandle_ = nullptr;
    Serial.println("Video prefetch task create failed");
    return false;
  }
  return true;
}

void MediaDemo::startVideoPrefetch() {
  if (!sdReady_) return;
  // Reclaim PSRAM used by image-sequence cache before video playback.
  releaseStillCache();
  if (!initVideoQueue()) return;
  if (!ensureVideoTask()) return;

  videoPrefetchEnabled_ = false;
  videoSourceNeedsInit_ = true;
  resetVideoQueue();
  videoPrefetchEnabled_ = true;
}

void MediaDemo::stopVideoPrefetch() {
  videoPrefetchEnabled_ = false;
  videoSourceNeedsInit_ = true;
  resetVideoQueue();
}

void MediaDemo::videoTaskThunk(void* arg) {
  static_cast<MediaDemo*>(arg)->videoTaskLoop();
}

void MediaDemo::videoTaskLoop() {
  for (;;) {
    if (!videoPrefetchEnabled_) {
      closeDt16Stream();
      closeMjpgStream();
      closeVideoStream();
      vTaskDelay(pdMS_TO_TICKS(kVideoTaskIdleDelayMs));
      continue;
    }

    if (videoSourceNeedsInit_) {
      if (!initVideoSource()) {
        vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs));
        continue;
      }
      videoSourceNeedsInit_ = false;
    }

    if (videoQueueCountSnapshot() >= kVideoQueueDepth) {
      ++videoDiagProducerQueueFullWaits_;
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    uint16_t* slot = nullptr;
    if (!acquireVideoWriteSlot(slot) || !slot) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    bool ok = false;
    const uint32_t readStartUs = micros();
    if (videoUseDt16Stream_) {
      ok = readNextDt16StreamFrame(slot);
    } else if (videoUseMjpgStream_) {
      ok = readNextMjpgStreamFrame(slot);
    } else if (videoUseStream_) {
      ok = readNextVideoStreamFrame(slot);
    } else if (videoUseJpegSequence_) {
      ok = readNextJpegSequenceFrame(slot);
    } else {
      ok = readNextVideoSequenceFrame(slot);
    }
    const uint32_t readUs = micros() - readStartUs;

    if (ok) {
      ++videoDiagProducerFrames_;
      videoDiagProducerReadUsTotal_ += readUs;
      if (readUs > videoDiagProducerReadUsMax_) {
        videoDiagProducerReadUsMax_ = readUs;
      }
      commitVideoWriteSlot();
      continue;
    }

    ++videoDiagProducerRetries_;
    vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs));
  }
}

bool MediaDemo::initVideoSource() {
  closeDt16Stream();
  closeMjpgStream();
  closeVideoStream();
  videoUseDt16Stream_ = false;
  videoUseMjpgStream_ = false;
  videoUseStream_ = false;
  videoUseJpegSequence_ = false;
  videoWarned_ = false;
  videoLoadIndex_ = 0;
  videoFrameIntervalMs_ = kVideoFrameIntervalDefaultMs;

  if (openDt16Stream()) {
    videoUseDt16Stream_ = true;
    return true;
  }

  if (openMjpgStream()) {
    videoUseMjpgStream_ = true;
    return true;
  }

  if (openVideoStream()) {
    videoUseStream_ = true;
    return true;
  }

  char jpgPath[96] = {0};
  snprintf(jpgPath, sizeof(jpgPath), "%s/%04d.jpg", kDirVideo, 0);
  File jpg = SD_MMC.open(jpgPath, FILE_READ);
  if (jpg) {
    jpg.close();
    videoUseJpegSequence_ = true;
    Serial.printf("video: jpeg sequence %s/NNNN.jpg\n", kDirVideo);
    return true;
  }

  Serial.printf("video: fallback to %s/NNNN.r565 frames\n", kDirVideo);
  return true;
}

bool MediaDemo::openDt16Stream() {
  videoDt16StreamFile_ = SD_MMC.open(kPathVideoDt16Stream, FILE_READ);
  if (!videoDt16StreamFile_) {
    return false;
  }
  const size_t dt16FileBytes = static_cast<size_t>(videoDt16StreamFile_.size());
  if (dt16FileBytes < static_cast<size_t>(kDt16HeaderBytes + 4)) {
    closeDt16Stream();
    return false;
  }

  uint8_t header[kDt16HeaderBytes] = {0};
  if (videoDt16StreamFile_.read(header, sizeof(header)) != sizeof(header)) {
    closeDt16Stream();
    return false;
  }
  if (memcmp(header, "DT16", 4) != 0) {
    closeDt16Stream();
    return false;
  }

  const uint16_t w = static_cast<uint16_t>(header[4] | (header[5] << 8));
  const uint16_t h = static_cast<uint16_t>(header[6] | (header[7] << 8));
  const uint32_t frameCount = static_cast<uint32_t>(header[8]) |
                              (static_cast<uint32_t>(header[9]) << 8) |
                              (static_cast<uint32_t>(header[10]) << 16) |
                              (static_cast<uint32_t>(header[11]) << 24);
  const uint16_t fps = static_cast<uint16_t>(header[12] | (header[13] << 8));
  const uint16_t tileSize = static_cast<uint16_t>(header[14] | (header[15] << 8));

  if (w != DisplayCanvas::kWidth || h != DisplayCanvas::kHeight) {
    closeDt16Stream();
    return false;
  }
  if (tileSize == 0 || tileSize > 64 || (w % tileSize) != 0 || (h % tileSize) != 0) {
    Serial.printf("dt16 invalid tile size: %u for %ux%u\n",
                  static_cast<unsigned>(tileSize), static_cast<unsigned>(w),
                  static_cast<unsigned>(h));
    closeDt16Stream();
    return false;
  }
  if (frameCount == 0) {
    closeDt16Stream();
    return false;
  }

  if (!videoDt16StateFrame_) {
    videoDt16StateFrame_ = static_cast<uint16_t*>(heap_caps_malloc(
        kVideoFrameBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!videoDt16StateFrame_) {
      videoDt16StateFrame_ = static_cast<uint16_t*>(
          heap_caps_malloc(kVideoFrameBytes, MALLOC_CAP_8BIT));
    }
  }
  if (!videoDt16StateFrame_) {
    closeDt16Stream();
    return false;
  }
  memset(videoDt16StateFrame_, 0, kVideoFrameBytes);

  videoDt16FrameCount_ = frameCount;
  videoDt16NextFrame_ = 0;
  videoDt16TileSize_ = tileSize;
  videoDt16TilesX_ = static_cast<uint16_t>(w / tileSize);
  videoDt16TilesY_ = static_cast<uint16_t>(h / tileSize);
  videoDt16Preloaded_ = false;
  videoDt16Blob_ = nullptr;
  videoDt16BlobBytes_ = 0;
  videoDt16Pos_ = 0;

  if (dt16FileBytes <= kDt16PsrampreloadMaxBytes) {
    videoDt16Blob_ = static_cast<uint8_t*>(
        heap_caps_malloc(dt16FileBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (videoDt16Blob_) {
      videoDt16StreamFile_.seek(0);
      if (readExact(videoDt16StreamFile_, videoDt16Blob_, dt16FileBytes)) {
        videoDt16Preloaded_ = true;
        videoDt16BlobBytes_ = dt16FileBytes;
        videoDt16Pos_ = kDt16HeaderBytes;
        videoDt16StreamFile_.close();
        Serial.printf("video: dt16 preloaded to PSRAM (%lu bytes)\n",
                      static_cast<unsigned long>(dt16FileBytes));
      } else {
        heap_caps_free(videoDt16Blob_);
        videoDt16Blob_ = nullptr;
      }
    }
    if (!videoDt16Preloaded_) {
      Serial.printf("video: dt16 preload failed (%lu bytes), using SD stream\n",
                    static_cast<unsigned long>(dt16FileBytes));
    }
  } else {
    Serial.printf("video: dt16 too large for preload (%lu bytes > %lu)\n",
                  static_cast<unsigned long>(dt16FileBytes),
                  static_cast<unsigned long>(kDt16PsrampreloadMaxBytes));
  }

  if (!videoDt16Preloaded_) {
    videoDt16StreamFile_.seek(kDt16HeaderBytes);
  }

  if (fps > 0) {
    videoFrameIntervalMs_ = 1000UL / fps;
    if (videoFrameIntervalMs_ == 0) videoFrameIntervalMs_ = 1;
  } else {
    videoFrameIntervalMs_ = kVideoFrameIntervalDefaultMs;
  }

  Serial.printf(
      "video: dt16 %s frames=%lu fps=%u interval=%lums tile=%u\n",
      kPathVideoDt16Stream, static_cast<unsigned long>(videoDt16FrameCount_),
      static_cast<unsigned>(fps), static_cast<unsigned long>(videoFrameIntervalMs_),
      static_cast<unsigned>(videoDt16TileSize_));
  return true;
}

bool MediaDemo::openMjpgStream() {
  videoMjpgStreamFile_ = SD_MMC.open(kPathVideoMjpgStream, FILE_READ);
  if (!videoMjpgStreamFile_) {
    return false;
  }
  if (videoMjpgStreamFile_.size() < static_cast<size_t>(kMjpgHeaderBytes + 4)) {
    closeMjpgStream();
    return false;
  }

  uint8_t header[kMjpgHeaderBytes] = {0};
  if (videoMjpgStreamFile_.read(header, sizeof(header)) != sizeof(header)) {
    closeMjpgStream();
    return false;
  }
  if (memcmp(header, "MJPG", 4) != 0) {
    closeMjpgStream();
    return false;
  }

  const uint16_t w = static_cast<uint16_t>(header[4] | (header[5] << 8));
  const uint16_t h = static_cast<uint16_t>(header[6] | (header[7] << 8));
  const uint32_t frameCount = static_cast<uint32_t>(header[8]) |
                              (static_cast<uint32_t>(header[9]) << 8) |
                              (static_cast<uint32_t>(header[10]) << 16) |
                              (static_cast<uint32_t>(header[11]) << 24);
  const uint16_t fps = static_cast<uint16_t>(header[12] | (header[13] << 8));
  if (w == 0 || h == 0 || w > DisplayCanvas::kWidth || h > DisplayCanvas::kHeight) {
    Serial.printf("mjpg stream invalid size: %ux%u\n", w, h);
    closeMjpgStream();
    return false;
  }

  videoMjpgFrameCount_ = frameCount;
  videoMjpgNextFrame_ = 0;
  videoMjpgStreamFile_.seek(kMjpgHeaderBytes);

  if (fps >= 8 && fps <= 60) {
    videoFrameIntervalMs_ = 1000UL / fps;
  } else if (fps > 0) {
    videoFrameIntervalMs_ = 1000UL / fps;
  } else {
    videoFrameIntervalMs_ = kVideoFrameIntervalDefaultMs;
  }

  Serial.printf("video: mjpg %s frames=%lu fps=%u interval=%lums size=%ux%u\n",
                kPathVideoMjpgStream,
                static_cast<unsigned long>(videoMjpgFrameCount_),
                static_cast<unsigned>(fps),
                static_cast<unsigned long>(videoFrameIntervalMs_),
                static_cast<unsigned>(w), static_cast<unsigned>(h));
  return true;
}

bool MediaDemo::openVideoStream() {
  videoStreamFile_ = SD_MMC.open(kPathVideoStream, FILE_READ);
  if (!videoStreamFile_) {
    return false;
  }

  if (videoStreamFile_.size() < static_cast<size_t>(kVideoStreamHeaderBytes)) {
    closeVideoStream();
    return false;
  }

  uint8_t header[kVideoStreamHeaderBytes] = {0};
  if (videoStreamFile_.read(header, sizeof(header)) != sizeof(header)) {
    closeVideoStream();
    return false;
  }
  if (memcmp(header, "V565", 4) != 0) {
    closeVideoStream();
    return false;
  }

  const uint16_t w = static_cast<uint16_t>(header[4] | (header[5] << 8));
  const uint16_t h = static_cast<uint16_t>(header[6] | (header[7] << 8));
  uint32_t frameCount = static_cast<uint32_t>(header[8]) |
                        (static_cast<uint32_t>(header[9]) << 8) |
                        (static_cast<uint32_t>(header[10]) << 16) |
                        (static_cast<uint32_t>(header[11]) << 24);
  const uint16_t fps = static_cast<uint16_t>(header[12] | (header[13] << 8));

  if (w != DisplayCanvas::kWidth || h != DisplayCanvas::kHeight) {
    Serial.printf("video stream invalid size: %ux%u (expected 480x480)\n", w, h);
    closeVideoStream();
    return false;
  }

  const size_t payloadBytes =
      static_cast<size_t>(videoStreamFile_.size()) - kVideoStreamHeaderBytes;
  if (payloadBytes < kVideoFrameBytes) {
    closeVideoStream();
    return false;
  }

  const uint32_t payloadFrameCount =
      static_cast<uint32_t>(payloadBytes / kVideoFrameBytes);
  if (frameCount == 0 || frameCount > payloadFrameCount) {
    frameCount = payloadFrameCount;
  }
  if (frameCount == 0) {
    closeVideoStream();
    return false;
  }

  videoStreamFrameCount_ = frameCount;
  videoStreamNextFrame_ = 0;
  videoStreamFile_.seek(kVideoStreamHeaderBytes);

  if (fps >= 8 && fps <= 60) {
    videoFrameIntervalMs_ = 1000UL / fps;
  } else {
    videoFrameIntervalMs_ = kVideoFrameIntervalDefaultMs;
  }

  Serial.printf("video: stream %s frames=%lu fps=%u interval=%lums\n",
                kPathVideoStream, static_cast<unsigned long>(videoStreamFrameCount_),
                static_cast<unsigned>(fps),
                static_cast<unsigned long>(videoFrameIntervalMs_));
  return true;
}

void MediaDemo::closeDt16Stream() {
  if (videoDt16StreamFile_) {
    videoDt16StreamFile_.close();
  }
  if (videoDt16Blob_) {
    heap_caps_free(videoDt16Blob_);
    videoDt16Blob_ = nullptr;
  }
  videoDt16Preloaded_ = false;
  videoDt16BlobBytes_ = 0;
  videoDt16Pos_ = 0;
  videoDt16FrameCount_ = 0;
  videoDt16NextFrame_ = 0;
  videoDt16TileSize_ = 0;
  videoDt16TilesX_ = 0;
  videoDt16TilesY_ = 0;
  if (videoDt16StateFrame_) {
    heap_caps_free(videoDt16StateFrame_);
    videoDt16StateFrame_ = nullptr;
  }
}

void MediaDemo::closeVideoStream() {
  if (videoStreamFile_) {
    videoStreamFile_.close();
  }
  videoStreamFrameCount_ = 0;
  videoStreamNextFrame_ = 0;
}

void MediaDemo::closeMjpgStream() {
  if (videoMjpgStreamFile_) {
    videoMjpgStreamFile_.close();
  }
  videoMjpgFrameCount_ = 0;
  videoMjpgNextFrame_ = 0;
}

bool MediaDemo::dt16ReadBytes(void* dst, size_t bytes) {
  if (bytes == 0) {
    return true;
  }
  if (videoDt16Preloaded_) {
    if (!videoDt16Blob_ || (videoDt16Pos_ + bytes) > videoDt16BlobBytes_) {
      return false;
    }
    memcpy(dst, videoDt16Blob_ + videoDt16Pos_, bytes);
    videoDt16Pos_ += bytes;
    return true;
  }
  if (!videoDt16StreamFile_) {
    return false;
  }
  return readExact(videoDt16StreamFile_, static_cast<uint8_t*>(dst), bytes);
}

bool MediaDemo::dt16Seek(size_t pos) {
  if (videoDt16Preloaded_) {
    if (!videoDt16Blob_ || pos > videoDt16BlobBytes_) {
      return false;
    }
    videoDt16Pos_ = pos;
    return true;
  }
  if (!videoDt16StreamFile_) {
    return false;
  }
  return videoDt16StreamFile_.seek(pos);
}

void MediaDemo::dt16ResetToDataStart() { dt16Seek(kDt16HeaderBytes); }

bool MediaDemo::readNextDt16StreamFrame(uint16_t* dst) {
  if (!dst || (!videoDt16StreamFile_ && !videoDt16Preloaded_) || !videoDt16StateFrame_) {
    return false;
  }

  const size_t tilePixels =
      static_cast<size_t>(videoDt16TileSize_) * static_cast<size_t>(videoDt16TileSize_);
  const size_t tileBytes = tilePixels * sizeof(uint16_t);
  if (!ensureVideoCompressedBuffer(tileBytes)) {
    return false;
  }

  auto readFrame = [&]() -> bool {
    uint8_t fh[4] = {0};
    if (!dt16ReadBytes(fh, sizeof(fh))) {
      return false;
    }
    const uint16_t flags = static_cast<uint16_t>(fh[0] | (fh[1] << 8));
    const uint16_t tileCount = static_cast<uint16_t>(fh[2] | (fh[3] << 8));
    const bool keyframe = (flags & 0x0001u) != 0;

    if (keyframe) {
      if (!dt16ReadBytes(reinterpret_cast<uint8_t*>(videoDt16StateFrame_),
                         kVideoFrameBytes)) {
        return false;
      }
    } else {
      const uint32_t tileTotal =
          static_cast<uint32_t>(videoDt16TilesX_) * static_cast<uint32_t>(videoDt16TilesY_);
      for (uint16_t i = 0; i < tileCount; ++i) {
        uint8_t idb[2] = {0};
        if (!dt16ReadBytes(idb, sizeof(idb))) {
          return false;
        }
        const uint16_t tileIndex = static_cast<uint16_t>(idb[0] | (idb[1] << 8));
        if (tileIndex >= tileTotal) {
          return false;
        }
        if (!dt16ReadBytes(videoCompressedBuf_, tileBytes)) {
          return false;
        }

        const uint16_t tileX = static_cast<uint16_t>(tileIndex % videoDt16TilesX_);
        const uint16_t tileY = static_cast<uint16_t>(tileIndex / videoDt16TilesX_);
        const int dstX = static_cast<int>(tileX) * static_cast<int>(videoDt16TileSize_);
        const int dstY = static_cast<int>(tileY) * static_cast<int>(videoDt16TileSize_);
        const uint16_t* srcTile = reinterpret_cast<const uint16_t*>(videoCompressedBuf_);
        for (uint16_t row = 0; row < videoDt16TileSize_; ++row) {
          uint16_t* dstRow = videoDt16StateFrame_ +
                             (static_cast<size_t>(dstY + row) * DisplayCanvas::kWidth) +
                             dstX;
          memcpy(dstRow, srcTile + static_cast<size_t>(row) * videoDt16TileSize_,
                 static_cast<size_t>(videoDt16TileSize_) * sizeof(uint16_t));
        }
      }
    }

    memcpy(dst, videoDt16StateFrame_, kVideoFrameBytes);
    return true;
  };

  if (!readFrame()) {
    dt16ResetToDataStart();
    videoDt16NextFrame_ = 0;
    memset(videoDt16StateFrame_, 0, kVideoFrameBytes);
    if (!readFrame()) {
      if (!videoWarned_) {
        videoWarned_ = true;
        Serial.printf("video: dt16 read failed: %s\n", kPathVideoDt16Stream);
      }
      return false;
    }
  }

  videoWarned_ = false;
  if (videoDt16FrameCount_ > 0) {
    ++videoDt16NextFrame_;
    if (videoDt16NextFrame_ >= videoDt16FrameCount_) {
      videoDt16NextFrame_ = 0;
      dt16ResetToDataStart();
      memset(videoDt16StateFrame_, 0, kVideoFrameBytes);
    }
  }
  return true;
}

bool MediaDemo::readNextMjpgStreamFrame(uint16_t* dst) {
  if (!dst || !videoMjpgStreamFile_) {
    return false;
  }

  uint8_t lenBuf[4] = {0};
  if (videoMjpgStreamFile_.read(lenBuf, sizeof(lenBuf)) != sizeof(lenBuf)) {
    videoMjpgStreamFile_.seek(kMjpgHeaderBytes);
    videoMjpgNextFrame_ = 0;
    if (videoMjpgStreamFile_.read(lenBuf, sizeof(lenBuf)) != sizeof(lenBuf)) {
      return false;
    }
  }

  const uint32_t jpegLen = static_cast<uint32_t>(lenBuf[0]) |
                           (static_cast<uint32_t>(lenBuf[1]) << 8) |
                           (static_cast<uint32_t>(lenBuf[2]) << 16) |
                           (static_cast<uint32_t>(lenBuf[3]) << 24);
  if (jpegLen == 0 || jpegLen > (2 * 1024 * 1024UL)) {
    if (!videoWarned_) {
      videoWarned_ = true;
      Serial.printf("video: invalid mjpg frame len=%lu\n",
                    static_cast<unsigned long>(jpegLen));
    }
    return false;
  }
  if (!ensureVideoCompressedBuffer(jpegLen)) {
    if (!videoWarned_) {
      videoWarned_ = true;
      Serial.println("video: compressed buffer alloc failed");
    }
    return false;
  }
  if (!readExact(videoMjpgStreamFile_, videoCompressedBuf_, jpegLen)) {
    videoMjpgStreamFile_.seek(kMjpgHeaderBytes);
    videoMjpgNextFrame_ = 0;
    return false;
  }
  if (!decodeJpegToFrame(videoCompressedBuf_, jpegLen, dst, true)) {
    if (!videoWarned_) {
      videoWarned_ = true;
      Serial.printf("video: mjpg decode failed frame=%lu\n",
                    static_cast<unsigned long>(videoMjpgNextFrame_));
    }
    return false;
  }

  videoWarned_ = false;
  if (videoMjpgFrameCount_ > 0) {
    ++videoMjpgNextFrame_;
    if (videoMjpgNextFrame_ >= videoMjpgFrameCount_) {
      videoMjpgNextFrame_ = 0;
      videoMjpgStreamFile_.seek(kMjpgHeaderBytes);
    }
  }
  return true;
}

bool MediaDemo::readNextVideoStreamFrame(uint16_t* dst) {
  if (!dst || !videoStreamFile_) {
    return false;
  }

  if (!readExactVideoFramePayload(videoStreamFile_, reinterpret_cast<uint8_t*>(dst),
                                  kVideoFrameBytes)) {
    videoStreamFile_.seek(kVideoStreamHeaderBytes);
    videoStreamNextFrame_ = 0;
    if (!readExactVideoFramePayload(videoStreamFile_,
                                    reinterpret_cast<uint8_t*>(dst),
                                    kVideoFrameBytes)) {
      if (!videoWarned_) {
        videoWarned_ = true;
        Serial.printf("video: stream read failed: %s\n", kPathVideoStream);
      }
      return false;
    }
  }

  videoWarned_ = false;
  if (videoStreamFrameCount_ > 0) {
    ++videoStreamNextFrame_;
    if (videoStreamNextFrame_ >= videoStreamFrameCount_) {
      videoStreamNextFrame_ = 0;
      videoStreamFile_.seek(kVideoStreamHeaderBytes);
    }
  }
  return true;
}

bool MediaDemo::ensureVideoReadStageBuffer(size_t bytes) {
  if (videoReadStageBuf_ && videoReadStageBufBytes_ >= bytes) {
    return true;
  }
  if (videoReadStageBuf_) {
    heap_caps_free(videoReadStageBuf_);
    videoReadStageBuf_ = nullptr;
    videoReadStageBufBytes_ = 0;
  }

  videoReadStageBuf_ = static_cast<uint8_t*>(
      heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
  if (!videoReadStageBuf_) {
    videoReadStageBuf_ = static_cast<uint8_t*>(
        heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  }
  if (!videoReadStageBuf_) {
    return false;
  }
  videoReadStageBufBytes_ = bytes;
  return true;
}

bool MediaDemo::readExactVideoFramePayload(File& f, uint8_t* dst, size_t bytes) {
  if (!dst) {
    return false;
  }

  if (!ensureVideoReadStageBuffer(kVideoReadStageChunkBytes)) {
    return readExact(f, dst, bytes);
  }

  size_t total = 0;
  while (total < bytes) {
    const size_t chunk =
        min(kVideoReadStageChunkBytes, static_cast<size_t>(bytes - total));
    const int n = f.read(videoReadStageBuf_, chunk);
    if (n <= 0) {
      return false;
    }
    memcpy(dst + total, videoReadStageBuf_, static_cast<size_t>(n));
    total += static_cast<size_t>(n);
  }
  return true;
}

bool MediaDemo::ensureVideoCompressedBuffer(size_t bytes) {
  if (videoCompressedBuf_ && videoCompressedBufBytes_ >= bytes) {
    return true;
  }
  if (videoCompressedBuf_) {
    heap_caps_free(videoCompressedBuf_);
    videoCompressedBuf_ = nullptr;
    videoCompressedBufBytes_ = 0;
  }

  videoCompressedBuf_ = static_cast<uint8_t*>(
      heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!videoCompressedBuf_) {
    videoCompressedBuf_ = static_cast<uint8_t*>(
        heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
  }
  if (!videoCompressedBuf_) {
    return false;
  }
  videoCompressedBufBytes_ = bytes;
  return true;
}

bool MediaDemo::decodeJpegToFrame(const uint8_t* data, size_t len, uint16_t* dst,
                                  bool clearBackground) {
  if (!data || len == 0 || !dst) {
    return false;
  }
  if (clearBackground) {
    memset(dst, 0, kVideoFrameBytes);
  }

  uint8_t* work = static_cast<uint8_t*>(
      heap_caps_malloc(kJpegDecodeWorkBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (!work) {
    work = static_cast<uint8_t*>(heap_caps_malloc(kJpegDecodeWorkBytes, MALLOC_CAP_8BIT));
  }
  if (!work) {
    return false;
  }

  esp_rom_tjpgd_dec_t dec = {};
  JpegDecodeSession session = {};
  session.reader.data = data;
  session.reader.len = len;
  session.reader.pos = 0;
  session.target.dst = dst;

  const esp_rom_tjpgd_result_t prep =
      esp_rom_tjpgd_prepare(&dec, tjpgdInputMem, work, kJpegDecodeWorkBytes, &session);
  if (prep != JDR_OK) {
    heap_caps_free(work);
    return false;
  }

  const int srcW = static_cast<int>(dec.width);
  const int srcH = static_cast<int>(dec.height);
  session.target.dstX = (DisplayCanvas::kWidth - srcW) / 2;
  session.target.dstY = (DisplayCanvas::kHeight - srcH) / 2;

  const esp_rom_tjpgd_result_t decomp =
      esp_rom_tjpgd_decomp(&dec, tjpgdOutputRgb888ToRgb565, 0);
  heap_caps_free(work);
  return decomp == JDR_OK;
}

bool MediaDemo::readNextJpegSequenceFrame(uint16_t* dst) {
  if (!dst) return false;

  char path[96] = {0};
  snprintf(path, sizeof(path), "%s/%04lu.jpg", kDirVideo,
           static_cast<unsigned long>(videoLoadIndex_));
  File f = SD_MMC.open(path, FILE_READ);
  if (!f && videoLoadIndex_ > 0) {
    videoLoadIndex_ = 0;
    snprintf(path, sizeof(path), "%s/%04lu.jpg", kDirVideo,
             static_cast<unsigned long>(videoLoadIndex_));
    f = SD_MMC.open(path, FILE_READ);
  }
  if (!f) {
    if (!videoWarned_) {
      videoWarned_ = true;
      Serial.printf("video: no readable jpeg frames in %s\n", kDirVideo);
    }
    return false;
  }

  const size_t jpegLen = static_cast<size_t>(f.size());
  if (jpegLen == 0 || jpegLen > (2 * 1024 * 1024UL) || !ensureVideoCompressedBuffer(jpegLen)) {
    f.close();
    return false;
  }
  if (!readExact(f, videoCompressedBuf_, jpegLen)) {
    f.close();
    return false;
  }
  f.close();

  if (!decodeJpegToFrame(videoCompressedBuf_, jpegLen, dst, true)) {
    if (!videoWarned_) {
      videoWarned_ = true;
      Serial.printf("video: jpeg decode failed: %s\n", path);
    }
    return false;
  }

  ++videoLoadIndex_;
  videoWarned_ = false;
  return true;
}

bool MediaDemo::readNextVideoSequenceFrame(uint16_t* dst) {
  if (!dst) return false;

  if (readVideoFrame(videoLoadIndex_, dst)) {
    videoLoadIndex_ += kVideoFrameStepFallback;
    videoWarned_ = false;
    return true;
  }

  if (videoLoadIndex_ > 0) {
    videoLoadIndex_ = 0;
    if (readVideoFrame(videoLoadIndex_, dst)) {
      videoLoadIndex_ += kVideoFrameStepFallback;
      videoWarned_ = false;
      return true;
    }
  }

  if (!videoWarned_) {
    videoWarned_ = true;
    Serial.printf("video: no readable frames in %s\n", kDirVideo);
  }
  return false;
}

bool MediaDemo::acquireVideoWriteSlot(uint16_t*& slot) {
  slot = nullptr;
  if (!videoQueueMutex_) {
    return false;
  }
  if (xSemaphoreTake(videoQueueMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return false;
  }
  if (videoQueueCount_ < kVideoQueueDepth) {
    slot = videoQueue_[videoQueueWriteIdx_];
  }
  xSemaphoreGive(videoQueueMutex_);
  return slot != nullptr;
}

void MediaDemo::commitVideoWriteSlot() {
  if (!videoQueueMutex_) {
    return;
  }
  if (xSemaphoreTake(videoQueueMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return;
  }
  if (videoQueueCount_ < kVideoQueueDepth) {
    videoQueueWriteIdx_ = (videoQueueWriteIdx_ + 1) % kVideoQueueDepth;
    ++videoQueueCount_;
    sampleVideoQueueDepth(videoQueueCount_);
  }
  xSemaphoreGive(videoQueueMutex_);
}

bool MediaDemo::popVideoReadSlot(uint16_t*& slot, size_t* remaining) {
  slot = nullptr;
  if (remaining) *remaining = 0;
  if (!videoQueueMutex_) {
    return false;
  }
  if (xSemaphoreTake(videoQueueMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return false;
  }
  if (videoQueueCount_ > 0) {
    slot = videoQueue_[videoQueueReadIdx_];
    videoQueueReadIdx_ = (videoQueueReadIdx_ + 1) % kVideoQueueDepth;
    --videoQueueCount_;
    sampleVideoQueueDepth(videoQueueCount_);
    if (remaining) *remaining = videoQueueCount_;
  }
  xSemaphoreGive(videoQueueMutex_);
  return slot != nullptr;
}

size_t MediaDemo::videoQueueCountSnapshot() {
  if (!videoQueueMutex_) {
    return 0;
  }
  if (xSemaphoreTake(videoQueueMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return 0;
  }
  const size_t c = videoQueueCount_;
  xSemaphoreGive(videoQueueMutex_);
  return c;
}

void MediaDemo::resetVideoDiagnostics(uint32_t nowMs) {
  videoDiagLastMs_ = nowMs;
  videoDiagQueueMin_ = 0;
  videoDiagQueueMax_ = 0;
  videoDiagProducerFrames_ = 0;
  videoDiagProducerReadUsTotal_ = 0;
  videoDiagProducerReadUsMax_ = 0;
  videoDiagProducerRetries_ = 0;
  videoDiagProducerQueueFullWaits_ = 0;
  videoDiagPresentFrames_ = 0;
  videoDiagPresentBlitUsTotal_ = 0;
  videoDiagPresentBlitUsMax_ = 0;
  videoDiagUnderflows_ = 0;
}

void MediaDemo::sampleVideoQueueDepth(size_t depth) {
  const uint32_t d = static_cast<uint32_t>(depth);
  if (d < videoDiagQueueMin_) {
    videoDiagQueueMin_ = d;
  }
  if (d > videoDiagQueueMax_) {
    videoDiagQueueMax_ = d;
  }
}

void MediaDemo::logVideoDiagnostics(uint32_t nowMs) {
  if (mode_ != Mode::kVideo) {
    return;
  }

  const uint32_t elapsed = nowMs - videoDiagLastMs_;
  if (elapsed < 1000) {
    return;
  }

  const size_t qNow = videoQueueCountSnapshot();
  sampleVideoQueueDepth(qNow);

  const uint32_t produced = videoDiagProducerFrames_;
  const uint32_t prodReadTotalUs = videoDiagProducerReadUsTotal_;
  const uint32_t prodReadMaxUs = videoDiagProducerReadUsMax_;
  const uint32_t prodRetries = videoDiagProducerRetries_;
  const uint32_t prodQueueFullWaits = videoDiagProducerQueueFullWaits_;
  const uint32_t presented = videoDiagPresentFrames_;
  const uint32_t presentBlitTotalUs = videoDiagPresentBlitUsTotal_;
  const uint32_t presentBlitMaxUs = videoDiagPresentBlitUsMax_;
  const uint32_t underflows = videoDiagUnderflows_;
  const uint32_t qMin = videoDiagQueueMin_;
  const uint32_t qMax = videoDiagQueueMax_;

  const uint32_t prodReadAvgUs = produced ? (prodReadTotalUs / produced) : 0;
  const uint32_t presentBlitAvgUs = presented ? (presentBlitTotalUs / presented) : 0;

  Serial.printf(
      "video diag: q=%u min=%u max=%u under=%lu prod=%lu pres=%lu load_us(avg/max)=%lu/%lu draw_us(avg/max)=%lu/%lu fullwait=%lu retry=%lu target=%lums\n",
      static_cast<unsigned>(qNow), static_cast<unsigned>(qMin),
      static_cast<unsigned>(qMax), static_cast<unsigned long>(underflows),
      static_cast<unsigned long>(produced), static_cast<unsigned long>(presented),
      static_cast<unsigned long>(prodReadAvgUs),
      static_cast<unsigned long>(prodReadMaxUs),
      static_cast<unsigned long>(presentBlitAvgUs),
      static_cast<unsigned long>(presentBlitMaxUs),
      static_cast<unsigned long>(prodQueueFullWaits),
      static_cast<unsigned long>(prodRetries),
      static_cast<unsigned long>(videoFrameIntervalMs_));

  videoDiagLastMs_ = nowMs;
  videoDiagQueueMin_ = static_cast<uint32_t>(qNow);
  videoDiagQueueMax_ = static_cast<uint32_t>(qNow);
  videoDiagProducerFrames_ = 0;
  videoDiagProducerReadUsTotal_ = 0;
  videoDiagProducerReadUsMax_ = 0;
  videoDiagProducerRetries_ = 0;
  videoDiagProducerQueueFullWaits_ = 0;
  videoDiagPresentFrames_ = 0;
  videoDiagPresentBlitUsTotal_ = 0;
  videoDiagPresentBlitUsMax_ = 0;
  videoDiagUnderflows_ = 0;
}

bool MediaDemo::readR565Frame(const char* path, uint16_t* dst,
                              bool requireFullScreen) {
  if (!path || !dst) return false;

  File f = SD_MMC.open(path, FILE_READ);
  if (!f) {
    return false;
  }

  uint16_t w = 0;
  uint16_t h = 0;
  if (!readHeader(f, "R565", w, h)) {
    f.close();
    return false;
  }
  if (requireFullScreen &&
      (w != DisplayCanvas::kWidth || h != DisplayCanvas::kHeight)) {
    f.close();
    Serial.printf("Frame not 480x480: %s (%ux%u)\n", path, w, h);
    return false;
  }
  const size_t bytes = static_cast<size_t>(w) * h * sizeof(uint16_t);
  if (!readExact(f, reinterpret_cast<uint8_t*>(dst), bytes)) {
    f.close();
    return false;
  }

  f.close();
  return true;
}

bool MediaDemo::readVideoFrame(uint32_t frameIndex, uint16_t* dst) {
  char path[96] = {0};
  snprintf(path, sizeof(path), "%s/%04lu.r565", kDirVideo,
           static_cast<unsigned long>(frameIndex));
  return readR565Frame(path, dst, true);
}

bool MediaDemo::fillVideoQueue() {
  return videoQueueCountSnapshot() > 0;
}

bool MediaDemo::presentVideoFrameFromQueue(uint32_t nowMs) {
  if (!canvas_) {
    return false;
  }

  uint16_t* src = nullptr;
  size_t remaining = 0;
  if (!popVideoReadSlot(src, &remaining) || !src) {
    ++videoDiagUnderflows_;
    return false;
  }
  const uint32_t blitStartUs = micros();
  if (canvas_->backend() == DisplayCanvas::Backend::kPanelFrameBuffers &&
      canvas_->frameBuffer() && canvas_->frameBytes() == kVideoFrameBytes) {
    memcpy(canvas_->frameBuffer(), src, kVideoFrameBytes);
    canvas_->present();
  } else {
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DisplayCanvas::kWidth,
                              DisplayCanvas::kHeight, src);
  }
  const uint32_t blitUs = micros() - blitStartUs;
  ++videoDiagPresentFrames_;
  videoDiagPresentBlitUsTotal_ += blitUs;
  if (blitUs > videoDiagPresentBlitUsMax_) {
    videoDiagPresentBlitUsMax_ = blitUs;
  }

  if (remaining <= 1 && !videoUseDt16Stream_ && !videoUseMjpgStream_ &&
      !videoUseStream_ && !videoUseJpegSequence_) {
    // When using per-frame files, ease pacing to reduce underflow.
    videoFrameIntervalMs_ = kVideoFrameIntervalDefaultMs + 20;
  }
  ++fpsFrames_;
  logFps("video", nowMs);
  return true;
}

bool MediaDemo::readExact(File& f, uint8_t* dst, size_t bytes) {
  size_t total = 0;
  while (total < bytes) {
    const int n = f.read(dst + total, bytes - total);
    if (n <= 0) {
      return false;
    }
    total += static_cast<size_t>(n);
  }
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
    if (!readExact(f, reinterpret_cast<uint8_t*>(canvas_->frameBuffer()), bytes)) {
      Serial.printf("Short read: %s\n", path);
      f.close();
      return false;
    }
  } else {
    if (!ensureScratch(bytes)) {
      f.close();
      return false;
    }
    if (!readExact(f, scratch_, bytes)) {
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

  if (!readExact(f, scratch_, bytes)) {
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

  if (!readExact(f, scratch_, bytes)) {
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

bool MediaDemo::loadStillSequenceFrame(uint32_t nowMs) {
  if (initStillCache()) {
    uint16_t* frame = stillCache_[sequenceIndex_ % kStillCacheDepth];
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DisplayCanvas::kWidth,
                              DisplayCanvas::kHeight, frame);
    ++sequenceIndex_;
    ++fpsFrames_;
    logFps("sequence", nowMs);
    return true;
  }

  const char* path = kStillSequence[sequenceIndex_ % kStillSequenceCount];
  if (!loadR565(path, true, true)) {
    if (!sequenceWarned_) {
      sequenceWarned_ = true;
      Serial.printf("sequence: missing frame %s\n", path);
    }
    return false;
  }

  sequenceWarned_ = false;
  ++sequenceIndex_;
  ++fpsFrames_;
  logFps("sequence", nowMs);
  return true;
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
  nextFrameDueMs_ = nowMs;
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
