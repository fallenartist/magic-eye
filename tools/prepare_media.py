#!/usr/bin/env python3

import argparse
import glob
import io
import math
import os
import pathlib
import struct
import subprocess
import sys
import tempfile

from PIL import Image, ImageOps
from PIL import ImageDraw


def ensure_parent(path: pathlib.Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def fit_image(img: Image.Image, width: int, height: int, fit: str, keep_alpha: bool) -> Image.Image:
    if fit == "cover":
        resample = Image.Resampling.LANCZOS
        return ImageOps.fit(img, (width, height), method=resample)

    if keep_alpha:
        canvas = Image.new("RGBA", (width, height), (0, 0, 0, 0))
        contained = ImageOps.contain(img, (width, height), method=Image.Resampling.LANCZOS)
        x = (width - contained.width) // 2
        y = (height - contained.height) // 2
        canvas.paste(contained, (x, y), contained if contained.mode in ("LA", "RGBA") else None)
        return canvas

    canvas = Image.new("RGB", (width, height), (0, 0, 0))
    contained = ImageOps.contain(img.convert("RGB"), (width, height), method=Image.Resampling.LANCZOS)
    x = (width - contained.width) // 2
    y = (height - contained.height) // 2
    canvas.paste(contained, (x, y))
    return canvas


def write_r565(img: Image.Image, output_path: pathlib.Path) -> None:
    rgb = img.convert("RGB")
    width, height = rgb.size
    ensure_parent(output_path)
    with output_path.open("wb") as f:
        f.write(b"R565")
        f.write(struct.pack("<HH", width, height))
        for r, g, b in rgb.getdata():
            value = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            f.write(struct.pack("<H", value))


def read_r565(path: pathlib.Path) -> tuple[int, int, bytes]:
    with path.open("rb") as f:
        header = f.read(8)
        if len(header) != 8 or header[:4] != b"R565":
            raise RuntimeError(f"Invalid R565 file: {path}")
        width, height = struct.unpack("<HH", header[4:8])
        expected = width * height * 2
        payload = f.read()
        if len(payload) != expected:
            raise RuntimeError(
                f"Invalid R565 payload size in {path}: got {len(payload)}, expected {expected}"
            )
        return width, height, payload


def write_v565_header(f, width: int, height: int, frame_count: int, fps: int) -> None:
    f.write(b"V565")
    f.write(struct.pack("<HHIHH", width, height, frame_count, fps, 0))


def write_mjpg_header(f, width: int, height: int, frame_count: int, fps: int) -> None:
    f.write(b"MJPG")
    f.write(struct.pack("<HHIHH", width, height, frame_count, fps, 0))


def write_dt16_header(f, width: int, height: int, frame_count: int, fps: int, tile: int) -> None:
    f.write(b"DT16")
    f.write(struct.pack("<HHIHH", width, height, frame_count, fps, tile))


def write_v565_from_png_files(
    files: list[str],
    output_path: pathlib.Path,
    width: int,
    height: int,
    fit: str,
    fps: int,
) -> None:
    ensure_parent(output_path)
    with output_path.open("wb") as out:
        write_v565_header(out, width, height, len(files), fps)
        for in_file in files:
            img = Image.open(in_file)
            fitted = fit_image(img, width, height, fit, keep_alpha=False)
            rgb = fitted.convert("RGB")
            for r, g, b in rgb.getdata():
                value = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
                out.write(struct.pack("<H", value))


def write_mjpg_from_png_files(
    files: list[str],
    output_path: pathlib.Path,
    width: int,
    height: int,
    fit: str,
    fps: int,
    quality: int,
) -> None:
    ensure_parent(output_path)
    with output_path.open("wb") as out:
        write_mjpg_header(out, width, height, len(files), fps)
        for in_file in files:
            img = Image.open(in_file)
            fitted = fit_image(img, width, height, fit, keep_alpha=False)
            rgb = fitted.convert("RGB")
            buf = io.BytesIO()
            rgb.save(buf, format="JPEG", quality=quality, optimize=False, progressive=False)
            payload = buf.getvalue()
            out.write(struct.pack("<I", len(payload)))
            out.write(payload)


def write_a155(img: Image.Image, output_path: pathlib.Path) -> None:
    rgba = img.convert("RGBA")
    width, height = rgba.size
    ensure_parent(output_path)
    with output_path.open("wb") as f:
        f.write(b"A155")
        f.write(struct.pack("<HH", width, height))
        for r, g, b, a in rgba.getdata():
            value = ((1 if a >= 128 else 0) << 15) | ((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3)
            f.write(struct.pack("<H", value))


def write_a856(img: Image.Image, output_path: pathlib.Path) -> None:
    rgba = img.convert("RGBA")
    width, height = rgba.size
    ensure_parent(output_path)
    with output_path.open("wb") as f:
        f.write(b"A856")
        f.write(struct.pack("<HH", width, height))
        for r, g, b, a in rgba.getdata():
            value = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            f.write(bytes((a, value & 0xFF, (value >> 8) & 0xFF)))


def encode_image_r565(args: argparse.Namespace) -> None:
    img = Image.open(args.input)
    fitted = fit_image(img, args.width, args.height, args.fit, keep_alpha=False)
    write_r565(fitted, pathlib.Path(args.output))
    print(f"Wrote {args.output}")


def encode_image_a155(args: argparse.Namespace) -> None:
    img = Image.open(args.input)
    fitted = fit_image(img, args.width, args.height, args.fit, keep_alpha=True)
    write_a155(fitted, pathlib.Path(args.output))
    print(f"Wrote {args.output}")


def encode_image_a856(args: argparse.Namespace) -> None:
    img = Image.open(args.input)
    fitted = fit_image(img, args.width, args.height, args.fit, keep_alpha=True)
    write_a856(fitted, pathlib.Path(args.output))
    print(f"Wrote {args.output}")


def encode_sequence_r565(args: argparse.Namespace) -> None:
    files = sorted(glob.glob(args.input_glob))
    if not files:
        raise RuntimeError(f"No files matched: {args.input_glob}")

    out_dir = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for i, in_file in enumerate(files):
        img = Image.open(in_file)
        fitted = fit_image(img, args.width, args.height, args.fit, keep_alpha=False)
        out_file = out_dir / f"{i:04d}.r565"
        write_r565(fitted, out_file)
    print(f"Wrote {len(files)} frame(s) to {out_dir}")


def encode_video_r565(args: argparse.Namespace) -> None:
    with tempfile.TemporaryDirectory(prefix="media_frames_") as tmp:
        pattern = os.path.join(tmp, "%06d.png")
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            args.input,
            "-vf",
            f"fps={args.fps}",
            "-start_number",
            "0",
            pattern,
        ]
        subprocess.run(cmd, check=True)
        seq_args = argparse.Namespace(
            input_glob=os.path.join(tmp, "*.png"),
            out_dir=args.out_dir,
            width=args.width,
            height=args.height,
            fit=args.fit,
        )
        encode_sequence_r565(seq_args)


def encode_video_v565(args: argparse.Namespace) -> None:
    with tempfile.TemporaryDirectory(prefix="media_frames_") as tmp:
        pattern = os.path.join(tmp, "%06d.png")
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            args.input,
            "-vf",
            f"fps={args.fps}",
            "-start_number",
            "0",
            pattern,
        ]
        subprocess.run(cmd, check=True)
        files = sorted(glob.glob(os.path.join(tmp, "*.png")))
        if not files:
            raise RuntimeError("No frames decoded from video")
        write_v565_from_png_files(
            files=files,
            output_path=pathlib.Path(args.output),
            width=args.width,
            height=args.height,
            fit=args.fit,
            fps=args.fps,
        )
        print(f"Wrote {args.output} ({len(files)} frame(s))")


def encode_video_mjpg(args: argparse.Namespace) -> None:
    with tempfile.TemporaryDirectory(prefix="media_frames_") as tmp:
        pattern = os.path.join(tmp, "%06d.png")
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            args.input,
            "-vf",
            f"fps={args.fps}",
            "-start_number",
            "0",
            pattern,
        ]
        subprocess.run(cmd, check=True)
        files = sorted(glob.glob(os.path.join(tmp, "*.png")))
        if not files:
            raise RuntimeError("No frames decoded from video")
        write_mjpg_from_png_files(
            files=files,
            output_path=pathlib.Path(args.output),
            width=args.width,
            height=args.height,
            fit=args.fit,
            fps=args.fps,
            quality=args.quality,
        )
        print(f"Wrote {args.output} ({len(files)} frame(s), JPEG q={args.quality})")


def encode_pack_v565(args: argparse.Namespace) -> None:
    files = sorted(glob.glob(args.input_glob))
    if not files:
        raise RuntimeError(f"No files matched: {args.input_glob}")

    first_w, first_h, first_payload = read_r565(pathlib.Path(files[0]))
    output_path = pathlib.Path(args.output)
    ensure_parent(output_path)
    with output_path.open("wb") as out:
        write_v565_header(out, first_w, first_h, len(files), args.fps)
        out.write(first_payload)
        for in_file in files[1:]:
            w, h, payload = read_r565(pathlib.Path(in_file))
            if w != first_w or h != first_h:
                raise RuntimeError(
                    f"Mismatched frame size in {in_file}: {w}x{h}, expected {first_w}x{first_h}"
                )
            out.write(payload)
    print(f"Wrote {args.output} ({len(files)} frame(s), {first_w}x{first_h})")


def encode_pack_mjpg(args: argparse.Namespace) -> None:
    files = sorted(glob.glob(args.input_glob))
    if not files:
        raise RuntimeError(f"No files matched: {args.input_glob}")

    output_path = pathlib.Path(args.output)
    ensure_parent(output_path)
    with output_path.open("wb") as out:
        first_w = 0
        first_h = 0
        write_mjpg_header(out, 0, 0, len(files), args.fps)  # placeholder; patched after first frame
        for i, in_file in enumerate(files):
            w, h, payload = read_r565(pathlib.Path(in_file))
            if i == 0:
                first_w, first_h = w, h
                out.seek(0)
                write_mjpg_header(out, first_w, first_h, len(files), args.fps)
                out.seek(0, os.SEEK_END)
            elif w != first_w or h != first_h:
                raise RuntimeError(
                    f"Mismatched frame size in {in_file}: {w}x{h}, expected {first_w}x{first_h}"
                )

            img = Image.frombytes("RGB", (w, h), b"".join(
                bytes((
                    ((px & 0xF800) >> 8) | ((px & 0xE000) >> 13),
                    ((px & 0x07E0) >> 3) | ((px & 0x0600) >> 9),
                    ((px & 0x001F) << 3) | ((px & 0x001C) >> 2),
                ))
                for (px,) in struct.iter_unpack("<H", payload)
            ))
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=args.quality, optimize=False, progressive=False)
            jpeg = buf.getvalue()
            out.write(struct.pack("<I", len(jpeg)))
            out.write(jpeg)

    print(
        f"Wrote {args.output} ({len(files)} frame(s), {first_w}x{first_h}, JPEG q={args.quality})"
    )


def _tile_bytes_r565(payload: bytes, width: int, tile: int, tx: int, ty: int) -> bytes:
    stride = width * 2
    row_bytes = tile * 2
    x0 = tx * row_bytes
    y0 = ty * tile
    return b"".join(
        payload[(y0 + row) * stride + x0 : (y0 + row) * stride + x0 + row_bytes]
        for row in range(tile)
    )


def _tile_changed_r565(prev_payload: bytes, payload: bytes, width: int, tile: int, tx: int, ty: int) -> bool:
    stride = width * 2
    row_bytes = tile * 2
    x0 = tx * row_bytes
    y0 = ty * tile
    for row in range(tile):
        start = (y0 + row) * stride + x0
        end = start + row_bytes
        if payload[start:end] != prev_payload[start:end]:
            return True
    return False


def encode_pack_dt16(args: argparse.Namespace) -> None:
    files = sorted(glob.glob(args.input_glob))
    if not files:
        raise RuntimeError(f"No files matched: {args.input_glob}")
    if args.max_frames is not None and args.max_frames > 0:
        files = files[: args.max_frames]
        if not files:
            raise RuntimeError("--max-frames filtered out all input frames")

    tile = args.tile
    if tile <= 0:
        raise RuntimeError("--tile must be > 0")

    first_w, first_h, first_payload = read_r565(pathlib.Path(files[0]))
    if first_w % tile != 0 or first_h % tile != 0:
        raise RuntimeError(f"Frame size {first_w}x{first_h} must be divisible by tile {tile}")

    tiles_x = first_w // tile
    tiles_y = first_h // tile
    total_tiles = tiles_x * tiles_y

    output_path = pathlib.Path(args.output)
    ensure_parent(output_path)

    keyframe_interval = max(0, args.keyframe_interval)
    total_changed_tiles = 0
    keyframes = 0
    prev_payload = first_payload

    with output_path.open("wb") as out:
      write_dt16_header(out, first_w, first_h, len(files), args.fps, tile)

      for i, in_file in enumerate(files):
          w, h, payload = read_r565(pathlib.Path(in_file))
          if w != first_w or h != first_h:
              raise RuntimeError(
                  f"Mismatched frame size in {in_file}: {w}x{h}, expected {first_w}x{first_h}"
              )

          is_keyframe = i == 0 or (keyframe_interval > 0 and (i % keyframe_interval) == 0)
          if is_keyframe:
              out.write(struct.pack("<HH", 0x0001, 0))
              out.write(payload)
              prev_payload = payload
              keyframes += 1
              continue

          changed: list[tuple[int, bytes]] = []
          for ty in range(tiles_y):
              for tx in range(tiles_x):
                  if _tile_changed_r565(prev_payload, payload, first_w, tile, tx, ty):
                      idx = ty * tiles_x + tx
                      changed.append((idx, _tile_bytes_r565(payload, first_w, tile, tx, ty)))

          out.write(struct.pack("<HH", 0x0000, len(changed)))
          for idx, tile_payload in changed:
              out.write(struct.pack("<H", idx))
              out.write(tile_payload)

          total_changed_tiles += len(changed)
          prev_payload = payload

    delta_frames = max(0, len(files) - keyframes)
    avg_changed = (total_changed_tiles / delta_frames) if delta_frames else 0.0
    pct = (100.0 * avg_changed / total_tiles) if total_tiles else 0.0
    print(
        f"Wrote {args.output} ({len(files)} frame(s), {first_w}x{first_h}, tile={tile}, "
        f"keyframes={keyframes}, avg_changed_tiles={avg_changed:.1f}/{total_tiles} ({pct:.1f}%))"
    )


def generate_demo_pack(args: argparse.Namespace) -> None:
    out_root = pathlib.Path(args.out_root)
    media = out_root / "media"
    (media / "sequence").mkdir(parents=True, exist_ok=True)
    (media / "video").mkdir(parents=True, exist_ok=True)

    # full.r565
    full = Image.new("RGB", (480, 480), (0, 0, 0))
    draw = ImageDraw.Draw(full)
    for y in range(480):
        c = int(255 * y / 479)
        draw.line([(0, y), (479, y)], fill=(c, 128, 255 - c))
    draw.ellipse((110, 110, 370, 370), outline=(255, 255, 255), width=8)
    draw.text((20, 20), "FULL", fill=(255, 255, 255))
    write_r565(full, media / "full.r565")

    # base.r565
    base = Image.new("RGB", (480, 480), (25, 25, 40))
    draw = ImageDraw.Draw(base)
    for i in range(0, 480, 20):
        draw.line([(i, 0), (479 - i, 479)], fill=(40, 80, 140), width=2)
    draw.text((18, 18), "BASE", fill=(255, 255, 255))
    write_r565(base, media / "base.r565")

    # round.a155
    round_img = Image.new("RGBA", (240, 240), (0, 0, 0, 0))
    draw = ImageDraw.Draw(round_img)
    draw.ellipse((0, 0, 239, 239), fill=(255, 190, 10, 255))
    draw.ellipse((34, 34, 206, 206), fill=(80, 30, 0, 255))
    draw.text((82, 108), "ROUND", fill=(255, 255, 255, 255))
    write_a155(round_img, media / "round.a155")

    # overlay.a155
    overlay = Image.new("RGBA", (480, 480), (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    draw.rectangle((60, 60, 420, 140), fill=(255, 40, 40, 255))
    draw.rectangle((90, 180, 390, 260), fill=(40, 200, 80, 255))
    draw.rectangle((120, 300, 360, 380), fill=(40, 140, 255, 255))
    draw.text((170, 92), "ALPHA", fill=(255, 255, 255, 255))
    write_a155(overlay, media / "overlay.a155")

    # sequence frames
    for i in range(args.sequence_frames):
        frame = Image.new("RGB", (480, 480), (0, 0, 0))
        draw = ImageDraw.Draw(frame)
        for y in range(480):
            c = (y + i * 7) % 255
            draw.line([(0, y), (479, y)], fill=(c, 60, 255 - c))
        x = (i * 13) % 480
        draw.ellipse((x - 40, 200, x + 40, 280), fill=(255, 255, 255))
        draw.text((15, 15), f"SEQ {i:04d}", fill=(255, 255, 0))
        write_r565(frame, media / "sequence" / f"{i:04d}.r565")

    # video frames
    for i in range(args.video_frames):
        frame = Image.new("RGB", (480, 480), (0, 0, 0))
        draw = ImageDraw.Draw(frame)
        cx = 240 + int(100 * math.sin(i * 0.11))
        cy = 240 + int(100 * math.cos(i * 0.09))
        r = 50 + (i % 40)
        draw.rectangle((0, 0, 479, 479), fill=(10, 10 + (i * 3) % 70, 30))
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=(255, 120, 20))
        draw.text((15, 15), f"VID {i:04d}", fill=(255, 255, 255))
        write_r565(frame, media / "video" / f"{i:04d}.r565")

    print(f"Wrote demo media pack to {media}")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prepare media files for ESP32-S3 demo")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("image-r565", help="Convert one image to R565 format")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--width", type=int, default=480)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fit", choices=("cover", "contain"), default="cover")
    p.set_defaults(func=encode_image_r565)

    p = sub.add_parser("image-a155", help="Convert one image to A155 format")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--width", type=int, default=240)
    p.add_argument("--height", type=int, default=240)
    p.add_argument("--fit", choices=("cover", "contain"), default="contain")
    p.set_defaults(func=encode_image_a155)

    p = sub.add_parser("image-a856", help="Convert one image to A856 format (8-bit alpha + RGB565)")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--width", type=int, default=240)
    p.add_argument("--height", type=int, default=240)
    p.add_argument("--fit", choices=("cover", "contain"), default="contain")
    p.set_defaults(func=encode_image_a856)

    p = sub.add_parser("seq-r565", help="Convert image sequence to R565 frames")
    p.add_argument("--input-glob", required=True)
    p.add_argument("--out-dir", required=True)
    p.add_argument("--width", type=int, default=480)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fit", choices=("cover", "contain"), default="cover")
    p.set_defaults(func=encode_sequence_r565)

    p = sub.add_parser("video-r565", help="Convert video to R565 frame sequence")
    p.add_argument("--input", required=True)
    p.add_argument("--out-dir", required=True)
    p.add_argument("--fps", type=int, default=24)
    p.add_argument("--width", type=int, default=480)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fit", choices=("cover", "contain"), default="cover")
    p.set_defaults(func=encode_video_r565)

    p = sub.add_parser("video-v565", help="Convert video to single streamed V565 file")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--fps", type=int, default=24)
    p.add_argument("--width", type=int, default=480)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fit", choices=("cover", "contain"), default="cover")
    p.set_defaults(func=encode_video_v565)

    p = sub.add_parser("video-mjpg", help="Convert video to single streamed MJPG file (JPEG frames)")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--fps", type=int, default=24)
    p.add_argument("--width", type=int, default=480)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fit", choices=("cover", "contain"), default="cover")
    p.add_argument("--quality", type=int, default=70, help="JPEG quality (1-95)")
    p.set_defaults(func=encode_video_mjpg)

    p = sub.add_parser("pack-v565", help="Pack existing R565 frame files into a single V565 file")
    p.add_argument("--input-glob", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--fps", type=int, default=24)
    p.set_defaults(func=encode_pack_v565)

    p = sub.add_parser("pack-mjpg", help="Pack existing R565 frame files into a single MJPG file")
    p.add_argument("--input-glob", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--fps", type=int, default=24)
    p.add_argument("--quality", type=int, default=70, help="JPEG quality (1-95)")
    p.set_defaults(func=encode_pack_mjpg)

    p = sub.add_parser("pack-dt16", help="Pack existing R565 frame files into a DT16 tile-delta stream")
    p.add_argument("--input-glob", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--fps", type=int, default=24)
    p.add_argument("--tile", type=int, default=16, help="Tile size in pixels (must divide frame size)")
    p.add_argument(
        "--keyframe-interval",
        type=int,
        default=0,
        help="Force periodic raw keyframes every N frames (0 = only first frame keyframe)",
    )
    p.add_argument("--max-frames", type=int, default=None, help="Limit number of frames packed")
    p.set_defaults(func=encode_pack_dt16)

    p = sub.add_parser("demo-pack", help="Generate synthetic demo media pack")
    p.add_argument("--out-root", required=True, help="Output root; pack will be created under <out-root>/media")
    p.add_argument("--sequence-frames", type=int, default=60)
    p.add_argument("--video-frames", type=int, default=120)
    p.set_defaults(func=generate_demo_pack)

    return parser


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()
    args.func(args)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as exc:
        print(f"ffmpeg failed: {exc}", file=sys.stderr)
        raise SystemExit(2)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
