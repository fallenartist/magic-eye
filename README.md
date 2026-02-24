# magic-eye

Pure ESP-IDF MJPEG/AVI playback experiment for the Waveshare ESP32-S3 2.1" round touch LCD (480x480).

## Status (Checkpoint: 2026-02-24)

- This branch is now cleaned for a pure ESP-IDF workflow (PlatformIO/Arduino app code removed).
- Active experiment app lives in `idf5_round_mjpeg/main/app_main.cpp`.
- Reused board bring-up drivers were moved into an ESP-IDF component:
  `idf5_round_mjpeg/components/waveshare_round_hw/`.
- Root `CMakeLists.txt` + `idf_component.yml` act as a repo-root wrapper so the ESP-IDF VS Code plugin can build from the repository root if desired.
- Local workspace/test asset folders (`.vscode/`, `media/`) are ignored to keep branch status clean.
- Build is working again in this branch (pure ESP-IDF app builds successfully).
- Fixed so far:
  - `SPI_MODE0` -> ESP-IDF-compatible SPI mode value in `Display_ST7701.cpp`
  - missing `main` component managed deps (`avi_player`, `esp_jpeg`) by adding `idf5_round_mjpeg/main/idf_component.yml` and `REQUIRES`
- Upstream-parity tweaks already applied:
  - `avi_player` buffer `512KB`
  - RGB panel `num_fbs = 2`
- New runtime behavior in `app_main.cpp`:
  - random playback of 1-second AVI clips from `/sdcard/media/video/random1s/`
  - touch toggles to `/sdcard/images/door-bell.a856` (or `/sdcard/media/images/door-bell.a856`)
  - next touch returns to random AVI playback
  - last frame stays visible while the next clip is opened (no clear between clips)
  - random clip directory scan validates `RIFF....AVI ` headers to skip macOS AppleDouble sidecar files (seen on FAT as names like `_CLIP_~1.AVI`)

## Measured Performance Conclusions (Important)

- With instrumented timings and a light `480x480 @ 15fps` MJPEG test file, the pipeline is **decode-bound**:
  - `decode ~= 163-167 ms/frame`
  - `draw ~= 29 ms/frame`
  - `total ~= 192-196 ms/frame`
  - sustained about `4.8-5.0 FPS`
- This means full-frame MJPEG (`esp_jpeg_decode + full RGB draw`) is not currently a realistic path to true `15 FPS` at `480x480`.
- Encoding lighter MJPEG helps, but only partially; decode cost remains dominant.
- Splitting into 1-second clips improves randomization/testing but adds additional clip start/parse stalls, which makes the low-FPS output feel more jittery.
- Final target content (drawn animation) may still perform better than photorealistic footage, but a custom format (delta/tile/palette or predecoded frames) is the likely path for hard `15 FPS`.

## Current MJPEG Decoder Tweaks Tried

- `esp_jpeg` switched to non-ROM decoder config in `idf5_round_mjpeg/sdkconfig.defaults`:
  - `CONFIG_JD_USE_ROM=n`
  - `CONFIG_JD_FASTDECODE_TABLE=y`
- Added persistent `esp_jpeg` working buffer (internal RAM) in `idf5_round_mjpeg/main/app_main.cpp`.
- On-device re-test showed **no material improvement** in decode time (`~165 ms/frame` remained).

## Why 15 FPS Is Not Achieved (Current Branch)

- `15 FPS` requires `<= 66.7 ms/frame` end-to-end.
- Measured current path is `~192-196 ms/frame` total.
- The main blocker is `esp_jpeg_decode()` (`~163-167 ms/frame`), not SD reads or DHT injection.
- `esp_lcd_panel_draw_bitmap()` is non-trivial (`~29 ms/frame`) but secondary.
- Result: current full-frame MJPEG AVI pipeline is throughput-limited to roughly `~5 FPS`.

## Build (ESP-IDF)

Option A (recommended): open `idf5_round_mjpeg/` as the ESP-IDF project folder in VS Code.

Option B: open repo root and use the wrapper `CMakeLists.txt`.

CLI example:

```bash
cd idf5_round_mjpeg
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/tty.wchusbserialXXXX flash monitor
```

## Video Input (AVI/MJPEG)

Firmware now prefers **random 1-second clips**:

- `/sdcard/media/video/random1s/*.avi`
- `/sdcard/media/video/random/*.avi`
- `/sdcard/videos/random1s/*.avi`

Fallback single-file paths:

- `/sdcard/media/video/video.avi`
- `/sdcard/videos/sample.avi`

Note on upstream sample:

- The commonly referenced upstream-style `sample.avi` is typically `24 FPS` MJPEG (not 15 FPS), even though some README guidance discusses 15 FPS encoding for custom clips.

### Current Test Clip Pack (Ready To Copy To SD)

Local generated test clips:

- `media/video/random1s/clip_000.avi` ... `clip_007.avi`

Encoding used:

```bash
ffmpeg -i media/eye-anim.mp4 -an -vf "fps=15,scale=480:480" -c:v mjpeg -q:v 40 \
  -f segment -segment_time 1 -reset_timestamps 1 -segment_format avi \
  media/video/random1s/clip_%03d.avi
```

Observed output (local):

- `8` clips, each `480x480`, `15 FPS`, `15` frames
- total size about `1,728,366 bytes`
- approx `14.4 KB/frame` including AVI segment overhead

Copy to SD (minimum for new behavior):

- `media/video/random1s/*.avi` -> `/sdcard/media/video/random1s/`
- `media/images/door-bell.a856` -> `/sdcard/images/door-bell.a856` (or `/sdcard/media/images/door-bell.a856`)

FatFS filename caveat:

- This project currently builds with `CONFIG_FATFS_LFN_NONE=y`, so long filenames are not guaranteed to appear on the ESP as-is.
- `clip_000.avi` is 8.3-safe and works.
- `door-bell.a856` is not 8.3-safe; firmware now scans for valid `A856` headers to find it via its FAT alias.

### Single AVI Conversion (Fallback)

Suggested conversion target:

```bash
ffmpeg -i input.mp4 -c:v mjpeg -q:v 2 -vf "fps=15,scale=480:480" -an video.avi
```

For throughput testing, use a much lighter MJPEG quality (high `q:v`, e.g. `31+`), not `q:v 2`.

## VS Code ESP-IDF Plugin Note

If you see:

- `Unexpected token '/' ... c_cpp_properties.json ... is not valid JSON`

that is usually a leftover PlatformIO-generated `.vscode/c_cpp_properties.json` (it contains `//` comments). Deleting that file fixes the ESP-IDF plugin configuration selection error.

## Touch Behavior (Current Experiment)

- Touch controller: `CST820` (polled over I2C)
- First touch: show `door-bell.a856` still image
- Next touch: return to random AVI clip mode
- If touch happens mid-clip, the clip parser continues but frame rendering is suppressed until toggled back (keeps logic simple and latency bounded by short clips)

## References

- Waveshare device wiki: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.1
- Upstream AVI player inspiration: https://github.com/KiranPranay/waveshare-esp32s3-4.3b-avi-player
