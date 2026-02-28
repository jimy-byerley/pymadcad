#!/usr/bin/env python3
"""Convert dark backgrounds in screenshots to transparent and re-export in place.

Usage:
    python docs/screenshot_remove_backgrounds.py [directory]

Defaults to docs/screenshots/ if no directory given.
"""

import sys
import glob
import os
import cv2
import numpy as np


def make_dark_transparent(path, low=30, high=60):
	"""Replace dark background pixels with transparency.

	Pixels with luminance <= low become fully transparent.
	Pixels with luminance between low and high get a smooth alpha ramp.
	Pixels with luminance >= high remain fully opaque.
	"""
	img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
	if img is None:
		print(f"  skipped (unreadable): {path}")
		return

	# convert to BGRA if needed
	if img.shape[2] == 3:
		img = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)

	# compute luminance from BGR channels
	b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
	luminance = 0.299 * r + 0.587 * g + 0.114 * b

	# smooth alpha ramp: 0 at low, 255 at high
	alpha = np.clip((luminance - low) / (high - low) * 255, 0, 255).astype(np.uint8)

	img[:, :, 3] = alpha
	cv2.imwrite(path, img)
	print(f"  done: {path}")


def main():
	if len(sys.argv) > 1:
		directory = sys.argv[1]
	else:
		directory = os.path.join(os.path.dirname(__file__), "screenshots")

	directory = os.path.abspath(directory)
	files = sorted(glob.glob(os.path.join(directory, "**", "*.png"), recursive=True))

	if not files:
		print(f"No PNG files found in {directory}")
		sys.exit(1)

	print(f"Processing {len(files)} PNG files in {directory}")
	for f in files:
		make_dark_transparent(f)
	print("All done.")


if __name__ == "__main__":
	main()
