#pragma once

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "DisplayCanvas.h"

class MediaDemo {
 public:
  enum class Mode : uint8_t {
    kFullImage = 0,
    kImageSequence,
    kRoundImage,
    kAlphaOverlay,
    kVideo,
    kCount
  };

  ~MediaDemo();
  bool begin(DisplayCanvas* canvas);
  void advanceMode();
  void update(uint32_t nowMs);
  const char* modeName() const;
  bool sdReady() const;

 private:
  static constexpr const char* kPathFull = "/media/images/eye-1.r565";
  static constexpr const char* kPathRoundA856 = "/media/images/door-bell.a856";
  static constexpr const char* kPathRoundA155 = "/media/images/door-bell.a155";
  static constexpr const char* kPathBase = "/media/images/eye-1.r565";
  static constexpr const char* kPathOverlayA856 = "/media/images/eye-alpha.a856";
  static constexpr const char* kPathOverlayA155 = "/media/images/eye-alpha.a155";
  static constexpr const char* kDirVideo = "/media/video";
  static constexpr const char* kPathVideoDt16Stream = "/media/video/video.dt16";
  static constexpr const char* kPathVideoMjpgStream = "/media/video/video.mjpg";
  static constexpr const char* kPathVideoStream = "/media/video/video.v565";
  static const char* const kStillSequence[];
  static constexpr size_t kStillSequenceCount = 4;

  DisplayCanvas* canvas_ = nullptr;
  bool sdReady_ = false;
  Mode mode_ = Mode::kFullImage;
  bool modeDirty_ = true;

  uint32_t sequenceIndex_ = 0;
  uint32_t videoIndex_ = 0;
  bool sequenceWarned_ = false;
  bool videoWarned_ = false;

  uint8_t* scratch_ = nullptr;
  size_t scratchBytes_ = 0;

  uint32_t fpsStartMs_ = 0;
  uint32_t fpsFrames_ = 0;
  uint32_t nextFrameDueMs_ = 0;
  volatile uint32_t videoFrameIntervalMs_ = 100;

  static constexpr uint32_t kSequenceFrameIntervalMs = 120;  // ~8 FPS
  static constexpr uint32_t kVideoFrameIntervalDefaultMs = 100;
  static constexpr uint32_t kRetryDelayMs = 150;
  static constexpr uint32_t kVideoFrameStep = 1;
  static constexpr uint32_t kVideoFrameStepFallback = 2;
  static constexpr size_t kVideoQueueDepth = 6;
  static constexpr size_t kVideoFillBurst = 2;
  static constexpr size_t kStillCacheDepth = 4;
  static constexpr uint32_t kVideoTaskIdleDelayMs = 8;
  static constexpr size_t kVideoStreamHeaderBytes = 16;

  uint16_t* videoQueue_[kVideoQueueDepth] = {};
  size_t videoQueueCount_ = 0;
  size_t videoQueueReadIdx_ = 0;
  size_t videoQueueWriteIdx_ = 0;
  uint32_t videoLoadIndex_ = 0;
  bool videoQueueReady_ = false;
  SemaphoreHandle_t videoQueueMutex_ = nullptr;
  TaskHandle_t videoTaskHandle_ = nullptr;
  volatile bool videoPrefetchEnabled_ = false;
  volatile bool videoSourceNeedsInit_ = true;
  bool videoUseDt16Stream_ = false;
  bool videoUseMjpgStream_ = false;
  bool videoUseStream_ = false;
  bool videoUseJpegSequence_ = false;
  File videoDt16StreamFile_;
  bool videoDt16Preloaded_ = false;
  uint8_t* videoDt16Blob_ = nullptr;
  size_t videoDt16BlobBytes_ = 0;
  size_t videoDt16Pos_ = 0;
  uint32_t videoDt16FrameCount_ = 0;
  uint32_t videoDt16NextFrame_ = 0;
  uint16_t videoDt16TileSize_ = 0;
  uint16_t videoDt16TilesX_ = 0;
  uint16_t videoDt16TilesY_ = 0;
  uint16_t* videoDt16StateFrame_ = nullptr;
  File videoMjpgStreamFile_;
  uint32_t videoMjpgFrameCount_ = 0;
  uint32_t videoMjpgNextFrame_ = 0;
  File videoStreamFile_;
  uint32_t videoStreamFrameCount_ = 0;
  uint32_t videoStreamNextFrame_ = 0;
  uint32_t videoDiagLastMs_ = 0;
  uint32_t videoDiagQueueMin_ = kVideoQueueDepth;
  uint32_t videoDiagQueueMax_ = 0;
  volatile uint32_t videoDiagProducerFrames_ = 0;
  volatile uint32_t videoDiagProducerReadUsTotal_ = 0;
  volatile uint32_t videoDiagProducerReadUsMax_ = 0;
  volatile uint32_t videoDiagProducerRetries_ = 0;
  volatile uint32_t videoDiagProducerQueueFullWaits_ = 0;
  volatile uint32_t videoDiagPresentFrames_ = 0;
  volatile uint32_t videoDiagPresentBlitUsTotal_ = 0;
  volatile uint32_t videoDiagPresentBlitUsMax_ = 0;
  volatile uint32_t videoDiagUnderflows_ = 0;
  uint8_t* videoReadStageBuf_ = nullptr;
  size_t videoReadStageBufBytes_ = 0;
  uint8_t* videoCompressedBuf_ = nullptr;
  size_t videoCompressedBufBytes_ = 0;
  uint16_t* stillCache_[kStillCacheDepth] = {nullptr, nullptr, nullptr, nullptr};
  bool stillCacheReady_ = false;

  bool initSdCard();
  bool ensureScratch(size_t bytes);
  bool initStillCache();
  void releaseStillCache();
  bool initVideoQueue();
  void resetVideoQueue();
  bool ensureVideoTask();
  void startVideoPrefetch();
  void stopVideoPrefetch();
  static void videoTaskThunk(void* arg);
  void videoTaskLoop();
  bool initVideoSource();
  bool openDt16Stream();
  bool openMjpgStream();
  bool openVideoStream();
  void closeDt16Stream();
  void closeMjpgStream();
  void closeVideoStream();
  bool readNextDt16StreamFrame(uint16_t* dst);
  bool readNextMjpgStreamFrame(uint16_t* dst);
  bool readNextVideoStreamFrame(uint16_t* dst);
  bool readNextJpegSequenceFrame(uint16_t* dst);
  bool readNextVideoSequenceFrame(uint16_t* dst);
  bool acquireVideoWriteSlot(uint16_t*& slot);
  void commitVideoWriteSlot();
  bool popVideoReadSlot(uint16_t*& slot, size_t* remaining);
  size_t videoQueueCountSnapshot();
  void resetVideoDiagnostics(uint32_t nowMs);
  void sampleVideoQueueDepth(size_t depth);
  void logVideoDiagnostics(uint32_t nowMs);
  bool ensureVideoCompressedBuffer(size_t bytes);
  bool ensureVideoReadStageBuffer(size_t bytes);
  bool dt16ReadBytes(void* dst, size_t bytes);
  bool dt16Seek(size_t pos);
  void dt16ResetToDataStart();
  bool readExactVideoFramePayload(File& f, uint8_t* dst, size_t bytes);
  bool decodeJpegToFrame(const uint8_t* data, size_t len, uint16_t* dst,
                         bool clearBackground);
  bool readR565Frame(const char* path, uint16_t* dst, bool requireFullScreen);
  bool readVideoFrame(uint32_t frameIndex, uint16_t* dst);
  bool fillVideoQueue();
  bool presentVideoFrameFromQueue(uint32_t nowMs);
  bool readExact(File& f, uint8_t* dst, size_t bytes);
  bool readHeader(File& f, const char expectedMagic[4], uint16_t& w, uint16_t& h);
  bool loadR565(const char* path, bool clearBackground, bool present);
  bool loadA155(const char* path, int dstX, int dstY, bool clearBackground,
                bool present);
  bool loadA856(const char* path, int dstX, int dstY, bool clearBackground,
                bool present);
  bool loadAlphaStill(const char* pathA856, const char* pathA155, int dstX,
                      int dstY, bool clearBackground, bool present);
  bool loadStillSequenceFrame(uint32_t nowMs);
  bool loadNextLoopedFrame(const char* directory, uint32_t& frameIndex,
                           bool& warnedNoFrames, const char* tag,
                           uint32_t nowMs);
  void resetCounters(uint32_t nowMs);
  void logFps(const char* tag, uint32_t nowMs);
};
