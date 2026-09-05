#!/usr/bin/env python3
"""Raster floor-plan line/loop extraction, for Roadmap Section 4
("Intelligent Auto Recognition") -- added 2026-08-27 after re-examining
the earlier "raster CV needs a trained ML model, not available" claim.

THAT CLAIM WAS TOO BROAD. It's true for the "recognize a door symbol in
a photo" kind of CV, which does need trained-model weights this offline
environment doesn't have. But extracting LINE SEGMENTS and CLOSED LOOPS
from a clean, line-drawing-style floor plan image is CLASSICAL computer
vision -- Canny edge detection + Hough line transform + contour finding
-- deterministic geometric algorithms, no neural network, no training
data, no GPU. OpenCV (opencv-python-headless) is genuinely available in
this environment and is used here for exactly that, nothing more.

PIPELINE: this script is the missing first stage that feeds
recognition::AutoRecognition's C++ classifiers (classifyAsColumn,
classifyAsWall, ...), which take vector line/loop data, not raster
pixels. This script produces that vector data FROM a raster image:
    raster image --[this script: Canny+Hough+contours]--> line segments
        and closed loops --[recognition::AutoRecognition, C++]--> Column/
        Wall/Door/Window classifications

SCOPE, STATED PLAINLY:
  - Works best on clean, line-drawing-style plans (CAD exports rendered
    to an image, scanned technical drawings) -- the kind of image where
    walls really are straight, high-contrast lines. A photo of a messy
    hand sketch, or a plan with furniture/text/dimension-line clutter,
    will produce noisy, false-positive-heavy output -- there is no
    learned model here to tell "wall line" apart from "furniture
    outline" by appearance, only by the downstream geometric
    classification (thickness, size, parallelism) AutoRecognition
    already does.
  - No scale/units detection -- output coordinates are in PIXELS, not
    meters. A caller must supply a pixels-per-meter scale factor
    (measured from a known dimension in the drawing) to convert before
    handing candidates to AutoRecognition, which expects meters.
  - This is a standalone Python tool (OpenCV has no equivalent in this
    project's C++ dependency set), producing a simple text output the
    C++ side can read -- not a compiled-in engine module.
"""

import argparse
import json
import sys

import cv2
import numpy as np


def extract_lines_and_loops(image_path, canny_low=50, canny_high=150,
                             hough_threshold=50, min_line_length=20,
                             max_line_gap=5, min_loop_area=100, max_loop_area=20000):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"could not read image: {image_path}")

    edges = cv2.Canny(img, canny_low, canny_high)

    # Hough probabilistic line transform -> individual line segments
    # (candidate wall edges, beam/column outlines).
    raw_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=hough_threshold,
                                 minLineLength=min_line_length, maxLineGap=max_line_gap)
    lines = []
    if raw_lines is not None:
        for l in raw_lines:
            x1, y1, x2, y2 = l[0]
            lines.append({"p1": [float(x1), float(y1)], "p2": [float(x2), float(y2)]})

    # Contour finding -> closed loops (candidate column cross-sections,
    # small rectangular shapes). RETR_LIST finds all contours regardless
    # of nesting; approxPolyDP simplifies each to its corner vertices so
    # a 4-vertex result is a candidate rectangle.
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    loops = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_loop_area or area > max_loop_area:
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) == 4:
            pts = [[float(p[0][0]), float(p[0][1])] for p in approx]
            loops.append(pts)

    return {"lines": lines, "loops": loops}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("image", help="path to the floor plan image")
    parser.add_argument("-o", "--output", help="output JSON path (default: stdout)")
    parser.add_argument("--canny-low", type=int, default=50)
    parser.add_argument("--canny-high", type=int, default=150)
    parser.add_argument("--hough-threshold", type=int, default=50)
    parser.add_argument("--min-line-length", type=int, default=20)
    parser.add_argument("--max-line-gap", type=int, default=5)
    parser.add_argument("--min-loop-area", type=float, default=100)
    parser.add_argument("--max-loop-area", type=float, default=20000)
    args = parser.parse_args()

    result = extract_lines_and_loops(
        args.image, args.canny_low, args.canny_high, args.hough_threshold,
        args.min_line_length, args.max_line_gap, args.min_loop_area, args.max_loop_area)

    out = json.dumps(result, indent=2)
    if args.output:
        with open(args.output, "w") as f:
            f.write(out)
    else:
        print(out)


if __name__ == "__main__":
    main()
