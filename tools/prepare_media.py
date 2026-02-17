#!/usr/bin/env python3

import argparse
import glob
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
