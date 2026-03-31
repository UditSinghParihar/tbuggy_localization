#!/usr/bin/env python3
"""
Convert webm screencasts to optimised GIFs for README embedding.

Usage:
  python3 utils/webm_to_gif.py <input.webm> <output.gif> [options]

Options:
  --width   Output width in pixels (default: 640)
  --fps     Output GIF frame rate (default: 8)
  --start   Start time in seconds (default: 0)
  --duration  Duration to capture in seconds (default: 12)
  --colors  Palette size 2-256 (default: 128)
"""

import argparse
import sys
import cv2
import numpy as np
from PIL import Image

def webm_to_gif(input_path, output_path, width=640, fps=8,
                start_sec=0.0, duration_sec=12.0, n_colors=128):
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        sys.exit(f"ERROR: cannot open {input_path}")

    src_fps_raw  = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # webm VFR reports 1000fps; use frame count + cap duration instead
    # Estimate real fps by reading a handful of timestamps
    if src_fps_raw >= 900:
        sample_pts = []
        for _ in range(200):
            ret, _ = cap.read()
            if not ret:
                break
            sample_pts.append(cap.get(cv2.CAP_PROP_POS_MSEC))
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        if len(sample_pts) >= 2:
            real_fps = (len(sample_pts) - 1) / ((sample_pts[-1] - sample_pts[0]) / 1000.0)
        else:
            real_fps = 30.0
        print(f"VFR webm detected. Estimated real fps: {real_fps:.1f}")
    else:
        real_fps = src_fps_raw

    # Frame sampling: pick every Nth frame to achieve target output fps
    frame_skip = max(1, round(real_fps / fps))
    start_frame = int(start_sec * real_fps)
    end_frame   = int((start_sec + duration_sec) * real_fps)

    print(f"Source: {src_w}x{src_h} @ {real_fps:.1f}fps, {total_frames} frames")
    print(f"Sampling frames {start_frame}–{end_frame}, every {frame_skip} frames")

    # Output dimensions
    scale = width / src_w
    height = int(src_h * scale)
    print(f"Output: {width}x{height} @ {fps}fps, palette={n_colors} colors")

    # Seek to start
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    frames_pil = []
    fnum = start_frame
    while fnum <= end_frame:
        ret, bgr = cap.read()
        if not ret:
            break
        if (fnum - start_frame) % frame_skip == 0:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb).resize((width, height), Image.LANCZOS)
            img = img.quantize(colors=n_colors, dither=1)
            frames_pil.append(img)
        fnum += 1

    cap.release()

    if not frames_pil:
        sys.exit("ERROR: no frames extracted")

    frame_duration_ms = int(1000 / fps)
    frames_pil[0].save(
        output_path,
        save_all=True,
        append_images=frames_pil[1:],
        loop=0,
        duration=frame_duration_ms,
        optimize=True,
    )

    import os
    size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"Saved {output_path}  ({len(frames_pil)} frames, {size_mb:.1f} MB)")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="webm -> optimised GIF")
    p.add_argument("input",  help="Input .webm file")
    p.add_argument("output", help="Output .gif file")
    p.add_argument("--width",    type=int,   default=640,  help="Output width px (default 640)")
    p.add_argument("--fps",      type=float, default=8,    help="Output fps (default 8)")
    p.add_argument("--start",    type=float, default=0.0,  help="Start time seconds (default 0)")
    p.add_argument("--duration", type=float, default=12.0, help="Duration seconds (default 12)")
    p.add_argument("--colors",   type=int,   default=128,  help="Palette colors 2-256 (default 128)")
    args = p.parse_args()
    webm_to_gif(args.input, args.output,
                width=args.width, fps=args.fps,
                start_sec=args.start, duration_sec=args.duration,
                n_colors=args.colors)
