# magic-eye

Door peephole using Waveshare ESP32-S3 2.1" round touch display.

## Project status

- PlatformIO Arduino project targeting Waveshare ESP32-S3 round LCD hardware.
- Current `src/` code is based on the manufacturer Sonnet_2-style driver path
  (`Display_ST7701` + `Touch_CST820`) with framebuffer-based rendering in `main.cpp`.
- `legacy_examples/` contains vendor/reference sketches.

## Quick start

1. Open this folder in VS Code with PlatformIO extension installed.
2. Make sure the USB serial driver for the board is installed and active (WCH CH34x/CH34xVCP).
3. Confirm the serial device exists (example): `/dev/tty.wchusbserial...` and `/dev/cu.wchusbserial...`.
4. Build/upload the default env: `waveshare_esp32s3_round`.
5. Open serial monitor at `115200`.

## Notes

- Upload is configured for reliability with this board/host combo:
  - `upload_speed = 57600`
  - `upload_flags = --no-stub`
- Memory profile is set for this display board:
  - `board_build.flash_mode = dio`
  - `board_build.psram_type = opi`
  - `board_build.arduino.memory_type = dio_opi`

## Media Demo

- Current firmware cycles demo modes on each touch:
  - `full_image` -> `image_sequence` -> `round_image` -> `alpha_overlay` -> `video`
- Media is read from SD card (`SD_MMC`) under `/media`.
- See `docs/media-prep.md` for required file layout and conversion commands.
