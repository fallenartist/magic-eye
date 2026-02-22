# Media Preparation

This firmware reads media files from SD card root at `/media`.

## File layout on SD card

```text
/media/full.r565
/media/base.r565
/media/round.a856        # preferred for smooth alpha (8-bit)
/media/round.a155
/media/overlay.a856      # preferred for smooth alpha (8-bit)
/media/overlay.a155
/media/sequence/0000.r565
/media/sequence/0001.r565
...
/media/video/video.v565    # preferred for smooth playback
/media/video/video.mjpg    # preferred for compressed video prototype (JPEG frames)
/media/video/video.dt16    # preferred for tile-delta prototype (eye animations)
/media/video/0000.jpg
/media/video/0001.jpg
/media/video/0000.r565
/media/video/0001.r565
...
```

## Formats

- `.r565`: header + RGB565 pixels
  - 4 bytes magic: `R565`
  - 2 bytes width (little-endian)
  - 2 bytes height (little-endian)
  - payload: `width * height * 2` bytes (`uint16_t` RGB565 LE)
- `.a155`: header + ARGB1555 pixels (1-bit alpha)
  - 4 bytes magic: `A155`
  - 2 bytes width (little-endian)
  - 2 bytes height (little-endian)
  - payload: `width * height * 2` bytes (`uint16_t` ARGB1555 LE)
  - alpha bit (`0x8000`) 1=opaque, 0=transparent
- `.a856`: header + 8-bit alpha + RGB565 (still images)
  - 4 bytes magic: `A856`
  - 2 bytes width (little-endian)
  - 2 bytes height (little-endian)
  - payload: `width * height * 3` bytes
  - each pixel: 1 byte alpha (`0..255`), then RGB565 LE (`uint16_t`)
- `.v565`: single streamed RGB565 video file
- `.mjpg`: single streamed JPEG-frame video file (custom container)
  - 4 bytes magic: `MJPG`
  - 2 bytes width, 2 bytes height
  - 4 bytes frame count
  - 2 bytes fps hint
  - 2 bytes reserved
  - payload: repeated `[4-byte jpeg_len][jpeg_bytes...]`
- `.dt16`: single streamed tile-delta RGB565 video file (custom container)
  - 4 bytes magic: `DT16`
  - 2 bytes width, 2 bytes height
  - 4 bytes frame count
  - 2 bytes fps hint
  - 2 bytes tile size (e.g. `16`)
  - payload: repeated frame records
  - frame record header: `u16 flags`, `u16 tile_count`
  - if `flags & 1` (keyframe): full raw RGB565 frame payload follows
  - else: `tile_count` entries of `[u16 tile_index][tile_rgb565_payload]`
  - 4 bytes magic: `V565`
  - 2 bytes width (little-endian)
  - 2 bytes height (little-endian)
  - 4 bytes frame count (little-endian, `0` allowed and treated as auto)
  - 2 bytes fps hint (little-endian)
  - 2 bytes reserved
  - payload: `frame_count * width * height * 2` bytes (raw RGB565 LE frames)

## Conversion tool

Install prerequisites:

```bash
python3 -m pip install pillow
brew install ffmpeg
```

Examples from project root:

```bash
# Full-screen image (480x480)
python3 tools/prepare_media.py image-r565 \
  --input assets/full.png \
  --output /Volumes/YOUR_SD/media/full.r565 \
  --width 480 --height 480 --fit cover

# Round/smaller image with smooth alpha (preferred)
python3 tools/prepare_media.py image-a856 \
  --input assets/round.png \
  --output /Volumes/YOUR_SD/media/round.a856 \
  --width 240 --height 240 --fit contain

# Fallback 1-bit alpha variant (optional)
python3 tools/prepare_media.py image-a155 \
  --input assets/round.png \
  --output /Volumes/YOUR_SD/media/round.a155 \
  --width 240 --height 240 --fit contain

# Base + overlay alpha composition
python3 tools/prepare_media.py image-r565 \
  --input assets/base.png \
  --output /Volumes/YOUR_SD/media/base.r565 \
  --width 480 --height 480 --fit cover

python3 tools/prepare_media.py image-a856 \
  --input assets/overlay.png \
  --output /Volumes/YOUR_SD/media/overlay.a856 \
  --width 480 --height 480 --fit contain

# Image sequence test
python3 tools/prepare_media.py seq-r565 \
  --input-glob "assets/sequence/*.png" \
  --out-dir /Volumes/YOUR_SD/media/sequence \
  --width 480 --height 480 --fit cover

# Video test (preferred: single streamed file)
python3 tools/prepare_media.py video-v565 \
  --input assets/video.mp4 \
  --output /Volumes/YOUR_SD/media/video/video.v565 \
  --fps 24 --width 480 --height 480 --fit cover

# Compressed video prototype (single streamed JPEG frames)
python3 tools/prepare_media.py video-mjpg \
  --input assets/video.mp4 \
  --output /Volumes/YOUR_SD/media/video/video.mjpg \
  --fps 24 --width 480 --height 480 --fit cover --quality 70

# Or pack existing frame files into a stream file
python3 tools/prepare_media.py pack-v565 \
  --input-glob "/Volumes/YOUR_SD/media/video/*.r565" \
  --output /Volumes/YOUR_SD/media/video/video.v565 \
  --fps 24

# Or pack existing R565 frame files into compressed MJPG stream
python3 tools/prepare_media.py pack-mjpg \
  --input-glob "/Volumes/YOUR_SD/media/video/*.r565" \
  --output /Volumes/YOUR_SD/media/video/video.mjpg \
  --fps 24 --quality 70

# Tile-delta prototype for mostly-static animations (e.g. eye)
python3 tools/prepare_media.py pack-dt16 \
  --input-glob "/Volumes/YOUR_SD/media/video/*.r565" \
  --output /Volumes/YOUR_SD/media/video/video.dt16 \
  --fps 24 --tile 8

# PSRAM preload test: shorter loop under ~8 MB
python3 tools/prepare_media.py pack-dt16 \
  --input-glob "/Volumes/YOUR_SD/media/video/*.r565" \
  --output /Volumes/YOUR_SD/media/video/video.dt16 \
  --fps 24 --tile 8 --max-frames 16

# Legacy option: convert video to frame sequence
python3 tools/prepare_media.py video-r565 \
  --input assets/video.mp4 \
  --out-dir /Volumes/YOUR_SD/media/video \
  --fps 24 --width 480 --height 480 --fit cover
```

## Performance notes

- `image_sequence` and `video` modes report measured FPS in serial logs.
- Full-size 480x480 RGB565 frame is ~450 KB, so frame rate depends mostly on SD read throughput + panel transfer.
- `video` mode now prefers `/media/video/video.dt16`, then `/media/video/video.mjpg`, then `/media/video/video.v565`.
- Fallback order then tries numbered `NNNN.jpg`, then `NNNN.r565`.
- DT16 playback attempts PSRAM preload for files up to ~8 MB (serial logs report preload success/fallback).
