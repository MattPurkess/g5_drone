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

import numpy as np
from PIL import Image
import argparse

from pupil_apriltags import Detector


def generate_tag36h11(tag_id: int, image_size_px: int = 800,
                       border_bits: int = 1) -> np.ndarray:
    """
    Render Tag36h11 tag_id as a numpy uint8 image of shape (image_size_px, image_size_px).
    border_bits: number of quiet-zone cells around the tag (min 1).
    """
    det = Detector(families='tag36h11')
    tf = det.tag_families['tag36h11']
    fam = tf.contents
    ncodes = int(fam.ncodes)

    if tag_id >= ncodes:
        raise ValueError(f'tag_id {tag_id} out of range (max {ncodes - 1})')

    # Tag36h11 layout:
    #   data_cells = 6  (6x6 = 36 data bits)
    #   tag_cells  = 8  (6 data + 1-cell black border on each side)
    #   total canvas cells = tag_cells + 2 * border_bits
    data_cells  = 6
    tag_cells   = 8
    total_cells = tag_cells + 2 * border_bits
    cell_px     = image_size_px // total_cells
    img_px      = cell_px * total_cells

    canvas = np.ones((img_px, img_px), dtype=np.uint8) * 255  # white background

    # 1- Draw outer quiet-zone border (border_bits wide, black)
    for bx in range(border_bits):
        canvas[bx * cell_px:(bx + 1) * cell_px, :] = 0
        canvas[img_px - (bx + 1) * cell_px:img_px - bx * cell_px, :] = 0
        canvas[:, bx * cell_px:(bx + 1) * cell_px] = 0
        canvas[:, img_px - (bx + 1) * cell_px:img_px - bx * cell_px] = 0

    # 2-Draw mandatory 1-cell black border of the tag itself
    offset       = border_bits          # cell index where the tag grid starts
    tag_px_start = offset * cell_px
    tag_px_end   = (offset + tag_cells) * cell_px

    canvas[tag_px_start:tag_px_start + cell_px, tag_px_start:tag_px_end] = 0  # top
    canvas[tag_px_end - cell_px:tag_px_end,     tag_px_start:tag_px_end] = 0  # bottom
    canvas[tag_px_start:tag_px_end, tag_px_start:tag_px_start + cell_px] = 0  # left
    canvas[tag_px_start:tag_px_end, tag_px_end - cell_px:tag_px_end]     = 0  # right

    # 3- Draw 6x6 data bits from the 36-bit stored code
    # fam.codes is a ctypes POINTER(c_ulong); convert to numpy array for safe indexing
    codes_array = np.ctypeslib.as_array(fam.codes, shape=(ncodes,))
    code         = int(codes_array[tag_id])
    data_offset  = offset + 1           # skip the 1-cell black tag border
    bit_index    = data_cells * data_cells - 1   # 35, MSB first

    for row in range(data_cells):
        for col in range(data_cells):
            ry = (data_offset + row) * cell_px
            rx = (data_offset + col) * cell_px
            bit    = (code >> bit_index) & 1
            colour = 255 if bit == 1 else 0   # white=1, black=0
            canvas[ry:ry + cell_px, rx:rx + cell_px] = colour
            bit_index -= 1

    return canvas


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