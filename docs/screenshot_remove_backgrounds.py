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


def detect_background(img, quantize=8):
	"""Detect the background color as the most represented value in the image.

	Colors are quantized into bins of size `quantize` to tolerate slight variations
	(e.g. anti-aliasing, compression artifacts). Returns the center of the most
	frequent bin as a BGR tuple.
	"""
	# use only the BGR channels
	pixels = img[:, :, :3].reshape(-1, 3).astype(np.int16)
	# quantize to bins
	binned = (pixels // quantize) * quantize + quantize // 2
	# pack into a single int per pixel for fast counting
	keys = binned[:, 0].astype(np.int32) | (binned[:, 1].astype(np.int32) << 8) | (binned[:, 2].astype(np.int32) << 16)
	values, counts = np.unique(keys, return_counts=True)
	best = values[np.argmax(counts)]
	# refine: average the actual pixel colors that fell into the winning bin
	mask = keys == best
	return pixels[mask].astype(np.float64).mean(axis=0)


def has_alpha_background(img, threshold_ratio=0.05):
	"""Detect whether the image already has a transparent background.

	Returns True if the image has an alpha channel and more than
	`threshold_ratio` of pixels are fully transparent (alpha == 0).
	"""
	if img.shape[2] < 4:
		return False
	transparent_count = np.count_nonzero(img[:, :, 3] == 0)
	total = img.shape[0] * img.shape[1]
	return transparent_count / total > threshold_ratio


def make_dark_transparent(path, low=5, high=100):
	"""Replace background pixels with transparency.

	The background color is detected as the most frequent color in the image.
	Pixels within `low` color-distance from the background become fully transparent.
	Between `low` and `high`, alpha ramps smoothly from 0 to 255.
	Beyond `high`, pixels remain fully opaque.
	Images that already have a transparent background are skipped.
	"""
	img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
	if img is None:
		print(f"  skipped (unreadable): {path}")
		return

	# convert to BGRA if needed
	if img.shape[2] == 3:
		img = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)

	# skip images that already have a transparent background
	if has_alpha_background(img):
		print(f"  skipped (already transparent): {path}")
		return

	# detect background color from histogram
	bg = detect_background(img)

	# compute per-pixel color distance to the background
	pixels = img[:, :, :3].astype(np.float32)
	distance = np.sqrt(np.sum((pixels - bg) ** 2, axis=2))

	# alpha from color distance: 0 below low, linear ramp to 1 at high
	alpha_f = np.clip((distance - low) / (high - low), 0.0, 1.0)

	# smooth alpha edges with a gaussian blur to propagate into transition pixels
	alpha_blurred = cv2.GaussianBlur(alpha_f, (0, 0), sigmaX=2.)
	# blur can only expand transparency, not reduce it
	alpha_f = np.minimum(alpha_f, alpha_blurred)

	# decontaminate edge colors: recover the original foreground by
	# removing the background contribution from anti-aliased pixels
	# pixel = alpha * fg + (1 - alpha) * bg  =>  fg = (pixel - (1-a)*bg) / a
	safe_alpha = np.maximum(alpha_f, 0.01)
	decontaminated = (pixels - (1.0 - alpha_f[:, :, np.newaxis]) * bg) / safe_alpha[:, :, np.newaxis]
	# we can make mistake in decontaminating that leads to overestimate the value
	error_factor = np.maximum(decontaminated.max(axis=2), 255)/255
	# increase alpha when error happens
	alpha_f = np.minimum(alpha_f * error_factor, 1)
	# lower decontaminated value with the correction, this has the effect of normalizing more the value
	decontaminated = decontaminated / error_factor[:,:,None]

	img[:, :, :3] = (decontaminated).astype(np.uint8)
	img[:, :, 3] = (alpha_f * 255).astype(np.uint8)
	cv2.imwrite(path, img)
	print(f"  done (bg={bg.astype(int).tolist()}): {path}")


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


def on_post_build(config, **kwargs):
	"""MkDocs hook: remove dark backgrounds from screenshots in the built site."""
	directory = os.path.join(config['site_dir'], "screenshots")
	files = sorted(glob.glob(os.path.join(directory, "**", "*.png"), recursive=True))
	if files:
		print(f"Processing {len(files)} PNG screenshots in {directory}")
		for f in files:
			make_dark_transparent(f)


if __name__ == "__main__":
	main()
