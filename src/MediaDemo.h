#pragma once

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>

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
  static constexpr const char* kPathFull = "/media/full.r565";
  static constexpr const char* kDirSequence = "/media/sequence";
  static constexpr const char* kPathRoundA856 = "/media/round.a856";
  static constexpr const char* kPathRoundA155 = "/media/round.a155";
  static constexpr const char* kPathBase = "/media/base.r565";
  static constexpr const char* kPathOverlayA856 = "/media/overlay.a856";
  static constexpr const char* kPathOverlayA155 = "/media/overlay.a155";
  static constexpr const char* kDirVideo = "/media/video";

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

  bool initSdCard();
  bool ensureScratch(size_t bytes);
  bool readHeader(File& f, const char expectedMagic[4], uint16_t& w, uint16_t& h);
  bool loadR565(const char* path, bool clearBackground, bool present);
  bool loadA155(const char* path, int dstX, int dstY, bool clearBackground,
                bool present);
  bool loadA856(const char* path, int dstX, int dstY, bool clearBackground,
                bool present);
  bool loadAlphaStill(const char* pathA856, const char* pathA155, int dstX,
                      int dstY, bool clearBackground, bool present);
  bool loadNextLoopedFrame(const char* directory, uint32_t& frameIndex,
                           bool& warnedNoFrames, const char* tag,
                           uint32_t nowMs);
  void resetCounters(uint32_t nowMs);
  void logFps(const char* tag, uint32_t nowMs);
};
