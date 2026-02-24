# Pure ESP-IDF Round MJPEG AVI Player (IDF 5.5+)

This is a pure ESP-IDF subproject for the `magic-eye` round display board (`480x480` ST7701 RGB panel on ESP32-S3).

Goals:
- Use **ESP-IDF tooling** (`idf.py` / ESP-IDF VS Code plugin), not PlatformIO
- Use **newer managed components** (matching the upstream Waveshare AVI player approach)
- Keep the existing round panel bring-up code, while moving the MJPEG pipeline closer to the upstream repo

## Libraries (Component Manager)

- `espressif/avi_player = 2.0.0`
- `espressif/esp_jpeg = 1.3.1`
- `idf >= 5.5`

## Build (ESP-IDF VS Code plugin or CLI)

1. Open `idf5_round_mjpeg/` as the project folder in the ESP-IDF VS Code plugin
2. Select target `esp32s3`
3. Build / Flash / Monitor

CLI equivalent:

```bash
cd idf5_round_mjpeg
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/tty.wchusbserialXXXX flash monitor
```

## SD Card Video Path

The player looks for:

- `/sdcard/media/video/video.avi`
- `/sdcard/videos/sample.avi`

## MJPEG AVI Conversion (Upstream-like, adapted to 480x480 round hardware)

The upstream 4.3" repo targets `15 FPS` MJPEG and explicitly warns that higher FPS inputs will play slow. For this board, start with the same strategy:

```bash
ffmpeg -i input.mp4 -c:v mjpeg -q:v 2 -vf "fps=15,scale=480:480" -an video.avi
```

Notes:
- `-q:v 2` favors fast decode (larger files, less compression)
- `fps=15` matches the upstream target style and is more realistic than `24/30 FPS` on ESP32-S3 software decode
- `-an` removes audio (this build is video-only)

## Status

This subproject is scaffolded for a pure-IDF migration and uses the newer component versions. It was not built in this terminal session because `idf.py` is not installed in the current environment.
