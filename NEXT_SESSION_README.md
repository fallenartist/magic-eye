# Next Session Handoff (Pure ESP-IDF MJPEG/AVI Experiment)

## Where We Left Off (2026-02-24)

- Goal remains the same: validate what is realistically possible for smooth playback on the Waveshare ESP32-S3 round display, and decide whether to keep MJPEG AVI or pivot.
- Branch was cleaned to remove legacy PlatformIO/Arduino app code.
- Board bring-up driver code was preserved and moved into:
  - `idf5_round_mjpeg/components/waveshare_round_hw/`
- Active app entrypoint:
  - `idf5_round_mjpeg/main/app_main.cpp`

## Most Important Findings (Performance)

- We added decode/draw timing instrumentation to the AVI callback and measured the bottleneck directly.
- On a light `480x480 @ 15fps` MJPEG AVI (roughly upstream-like bytes/frame), measured callback timings were:
  - `decode ~= 163-167 ms`
  - `draw ~= 29 ms`
  - `total ~= 192-196 ms`
  - sustained `~4.8-5.0 FPS`
- Conclusion: current full-frame MJPEG path is **decode-bound** (`esp_jpeg_decode`) and not close to `15 FPS` at `480x480`.
- This is not caused by mismatched core libraries; `avi_player` and `esp_jpeg` versions match upstream.
- Non-ROM `esp_jpeg` fastdecode + persistent working buffer were tested and did **not** materially improve the measured decode time (`~165 ms/frame` remained).
- Random 1-second clips are useful for behavior testing, but they add clip parse/start transitions and make the visual jitter more obvious at ~5 FPS.

## Cleanup Completed This Session

- Deleted legacy Arduino/PlatformIO sources and scaffolding:
  - old `src/` app tree
  - `platformio.ini`
  - old root `sdkconfig*.defaults`
  - PlatformIO placeholder dirs (`include/`, `lib/`, `test/`)
- Updated `idf5_round_mjpeg/main/CMakeLists.txt` to depend on `waveshare_round_hw` component
- Added component `idf5_round_mjpeg/components/waveshare_round_hw/CMakeLists.txt`
- Removed leftover Arduino branches from `I2C_Driver` in the new component
- Removed touch dependency from `Display_ST7701` for this pure video experiment path
- Deleted local `.vscode/c_cpp_properties.json` (PlatformIO-generated invalid JSON with comments) to fix ESP-IDF plugin configuration error
- Added `.vscode/` and `media/` to `.gitignore` to keep local workspace/test files out of branch status
- Removed root generated ESP-IDF files (`sdkconfig`, `esp_idf_project_configuration.json`)
- Ran `idf.py build` and fixed compile issues in `waveshare_round_hw`:
  - `SPI_MODE0` (Arduino-style) replaced with ESP-IDF-compatible mode value
- Added missing main-component managed dependency wiring:
  - `idf5_round_mjpeg/main/idf_component.yml`
  - `REQUIRES avi_player esp_jpeg` in `idf5_round_mjpeg/main/CMakeLists.txt`
- Added upstream-parity runtime tweaks:
  - `avi_player` buffer increased to `512KB`
  - RGB panel `num_fbs = 2`
- Added detailed callback timing logs (`prep/decode/draw/total/srcKB`) in `app_main.cpp`
- Added random AVI clip playback mode (1-second clips) from `/sdcard/media/video/random1s/`
- Added CST820 touch toggle:
  - touch -> show `door-bell.a856`
  - touch again -> return to random AVI mode
- Added `.a856` still-image rendering in pure IDF app (row-streamed alpha blend to RGB565 canvas)
- Added fallback scan for `door-bell.a856` in `/sdcard/images` and `/sdcard/media/images`
- Random clip scan now filters macOS AppleDouble sidecar files by validating AVI RIFF headers (fixes parse errors on `_CLIP_~*.AVI`)
- Added `esp_jpeg` persistent working buffer (internal RAM) and switched project config to non-ROM `esp_jpeg` fastdecode mode in `sdkconfig.defaults`
- Confirmed on-device that the above decoder tweak did not materially improve FPS (still decode-bound around ~5 FPS)
- Verified build success after these changes (including new runtime logic)

## Important Current Layout

- Repo root contains wrapper files for ESP-IDF plugin convenience:
  - `CMakeLists.txt`
  - `idf_component.yml`
- Standalone ESP-IDF project remains in:
  - `idf5_round_mjpeg/`
- Root wrapper now includes:
  - `idf5_round_mjpeg/main`
  - `idf5_round_mjpeg/components`

## SD Card Assets (Prepared Locally)

- Generated 1-second random clip set (ready to copy):
  - `media/video/random1s/clip_000.avi` ... `clip_007.avi`
  - each clip is `480x480`, `15 FPS`, `15` frames
- Touch still image asset already exists locally:
  - `media/images/door-bell.a856`

Copy to SD:

- `media/video/random1s/*.avi` -> `/sdcard/media/video/random1s/`
- `media/images/door-bell.a856` -> `/sdcard/images/door-bell.a856`

## Next Steps (Recommended)

1. Flash the latest build and capture logs after the new `CONFIG_JD_USE_ROM=n` + `FASTDECODE_TABLE` change.
2. Compare new timing logs (`decode` ms specifically) against the previous `~165 ms` baseline.
3. If still far from target (likely), decide whether to pivot:
   - custom drawn-animation format (tile/delta/palette)
   - predecoded short clips / hybrid approach
4. If staying on MJPEG briefly, test lower decode resolution (`out_scale`) as a hard limit probe.

## Known Risks / Open Questions

1. `15 FPS` full-frame `480x480` MJPEG remains unlikely on ESP32-S3 with current `esp_jpeg` path (decoder is the bottleneck).
2. Upstream repo visual smoothness claims are not instrumented the same way; direct FPS comparisons are ambiguous.
3. `avi_player` has no obvious stop API, so touch toggle currently suppresses rendering rather than stopping playback immediately (acceptable with 1-second clips).
4. This environment requires a build-time workaround for ESP-IDF gdbinit generation when `ESP_ROM_ELF_DIR` is unset (`ESP_ROM_ELF_DIR=/tmp`).
5. Project is currently built with `CONFIG_FATFS_LFN_NONE=y`; long filenames can appear as 8.3 aliases on the ESP (important for media naming like `door-bell.a856`).
6. `Display_ST7701` still carries upstream-style board init code (works for the experiment, but not yet refactored for cleanliness).

## Local / Generated Files

- `.vscode/settings.json` contains useful local ESP-IDF plugin settings (port, target).
- `media/` remains local/untracked test assets.
