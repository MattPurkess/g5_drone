#!/usr/bin/env python3
"""
Generate a Tag36h11 AprilTag PNG at a specified physical size.
The generated image is suitable for printing and for use as a Gazebo texture.

Tag36h11 structure:
  - 6x6 inner grid of data bits  (36 bits total)
  - 1-cell mandatory black border surrounding the data grid  (8x8 total tag area)
  - Optional quiet-zone (white) border of border_bits cells around the tag
"""

#Generate one tag for each drone!!

import argparse

import cv2
import numpy as np
from PIL import Image


def generate_tag36h11(tag_id: int, image_size_px: int = 800,
                       border_bits: int = 1) -> np.ndarray:
    """
    Render Tag36h11 tag_id as a numpy uint8 image of shape (image_size_px, image_size_px).
    border_bits: number of quiet-zone cells around the tag (min 1).
    """
    if tag_id < 0 or tag_id >= 587:
        raise ValueError('tag_id must be in range 0..586 for tag36h11')

    tag_cells = 8
    total_cells = tag_cells + 2 * border_bits
    cell_px = image_size_px // total_cells
    marker_px = cell_px * tag_cells
    quiet_px = cell_px * border_bits

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.drawMarker(dictionary, tag_id, marker_px)
    return cv2.copyMakeBorder(
        marker,
        quiet_px, quiet_px, quiet_px, quiet_px,
        cv2.BORDER_CONSTANT,
        value=255,
    )


def main():
    parser = argparse.ArgumentParser(description='Generate Tag36h11 AprilTag')
    parser.add_argument('--id',   type=int, default=0,             help='Tag ID (0-586)')
    parser.add_argument('--size', type=int, default=800,           help='Image size in pixels')
    parser.add_argument('--out',  type=str, default='apriltag.png', help='Output PNG path')
    args = parser.parse_args()

    img_array = generate_tag36h11(args.id, args.size)
    img = Image.fromarray(img_array, mode='L').convert('RGB')
    img.save(args.out)
    print(f'Saved Tag36h11 ID={args.id}  size={args.size}px  ->  {args.out}')


if __name__ == '__main__':
    main()
