#!/usr/bin/env python3
"""Flatten a 3D PCD map (e.g. from bebot_slam's loop closure node) into a 2D
occupancy grid (map.pgm + map.yaml) that nav2_map_server can serve.

There's no per-point sensor-origin/ray information saved in the PCD, so this
can't do proper occupancy-grid raycasting (which is how a "real" 2D SLAM map
distinguishes observed-free from never-observed). Instead it approximates:
  - "known" = a cell gets enough point hits (any height) -> the lidar
    reliably saw something here, so the area around it was observed.
  - "occupied" = a cell gets enough point hits *within the obstacle height
    band* -> a real surface is there, not just sensor noise.
  - known-but-not-occupied -> free. Never touched (or under threshold) ->
    unknown.
Point-count thresholds (rather than "any single point marks a cell") reject
most single-shot lidar noise before it ever reaches the grid. What density
thresholds can't fix is *ghost* walls/floors: if loop closure left residual
drift, the same real wall can show up twice, offset by a few cells, each
copy with plenty of points. Cleanup below (opening/closing + keeping only
the largest connected free-space blob) removes small ghost fragments and
disconnected free-space islands, but a solid, well-populated ghost wall
still reads as a real obstacle - the fix for that is tighter loop closure
upstream, not filtering here. Erring toward leaving a suspicious blob
occupied is intentional: a hallucinated wall costs the planner a detour, a
hallucinated opening costs a collision.

Usage:
  ros2 run bebot_navigation pcd_to_occupancy_grid --pcd ~/bebot_maps/bebot_map_loop_closed.pcd
"""

import argparse
import os

import numpy as np
from scipy.ndimage import binary_closing, binary_dilation, binary_opening, label


def read_pcd_ascii(path: str) -> np.ndarray:
    points = []
    with open(path, 'r') as f:
        in_data = False
        for line in f:
            if not in_data:
                if line.strip().upper().startswith('DATA'):
                    in_data = True
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            points.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return np.array(points, dtype=np.float64)


def estimate_floor_z(points: np.ndarray, bin_size: float = 0.02) -> float:
    """The floor is a large flat surface, so it dominates the Z histogram -
    find the tallest histogram peak in the lower half of the Z range."""
    z = points[:, 2]
    z_lo, z_hi = np.percentile(z, [1, 99])
    search_hi = z_lo + (z_hi - z_lo) * 0.5
    mask = (z >= z_lo) & (z <= search_hi)
    bins = np.arange(z_lo, search_hi + bin_size, bin_size)
    hist, edges = np.histogram(z[mask], bins=bins)
    peak_idx = int(np.argmax(hist))
    return float((edges[peak_idx] + edges[peak_idx + 1]) / 2.0)


def rasterize_counts(points: np.ndarray, resolution: float, obstacle_z_range):
    """Per-cell point counts (not booleans) so callers can threshold on
    density instead of "any single point marks the cell"."""
    xy = points[:, :2]
    z = points[:, 2]
    min_x, min_y = xy.min(axis=0) - resolution * 2
    max_x, max_y = xy.max(axis=0) + resolution * 2

    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))

    cols = ((xy[:, 0] - min_x) / resolution).astype(np.int64)
    rows = ((xy[:, 1] - min_y) / resolution).astype(np.int64)
    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    cols, rows, z = cols[valid], rows[valid], z[valid]
    flat_idx = rows * width + cols
    cell_count = width * height

    known_counts = np.bincount(flat_idx, minlength=cell_count).reshape(height, width)

    z_min, z_max = obstacle_z_range
    obstacle_mask = (z >= z_min) & (z <= z_max)
    occupied_counts = np.bincount(
        flat_idx[obstacle_mask], minlength=cell_count).reshape(height, width)

    return known_counts, occupied_counts, (min_x, min_y)


def remove_small_components(mask: np.ndarray, min_cells: int) -> np.ndarray:
    if min_cells <= 1:
        return mask
    labeled, num = label(mask)
    if num == 0:
        return mask
    sizes = np.bincount(labeled.ravel())
    sizes[0] = 0  # background label
    keep = sizes >= min_cells
    return keep[labeled]


def keep_largest_component(mask: np.ndarray) -> np.ndarray:
    labeled, num = label(mask)
    if num == 0:
        return mask
    sizes = np.bincount(labeled.ravel())
    sizes[0] = 0
    if sizes.max() == 0:
        return mask
    largest_label = int(np.argmax(sizes))
    return labeled == largest_label


def build_grid(points: np.ndarray, resolution: float, obstacle_z_range,
                min_known_points: int, min_occupied_points: int,
                opening_iterations: int, closing_iterations: int,
                min_blob_cells: int, known_dilate_cells: int,
                known_closing_iterations: int,
                keep_largest_free: bool):
    known_counts, occupied_counts, origin_xy = rasterize_counts(
        points, resolution, obstacle_z_range)

    known_grid = known_counts >= min_known_points
    occupied_grid = occupied_counts >= min_occupied_points

    # Opening (erosion+dilation) is too destructive here: a real 1-cell-wide
    # wall has no erosion-surviving neighborhood and gets wiped along with
    # the noise. Speckle removal instead relies on connected-component size
    # - a lone noisy point forms a component of size 1-2, a real wall or
    # shelf forms a long, large one, regardless of its 1-cell width.
    if opening_iterations > 0:
        occupied_grid = binary_opening(occupied_grid, iterations=opening_iterations)
    occupied_grid = remove_small_components(occupied_grid, min_blob_cells)
    # Fuse nearby wall fragments / close small sampling gaps in walls so the
    # global costmap can't find a path leaking through a 1-cell pinhole.
    if closing_iterations > 0:
        occupied_grid = binary_closing(occupied_grid, iterations=closing_iterations)

    known_grid = known_grid | occupied_grid
    # Closing first fills small unknown pockets *inside* the observed area
    # (sparse floor sampling reads as salt-and-pepper "unknown" speckle
    # otherwise) without growing the outer boundary much; the dilation after
    # it is just a small final pad for sampling gaps at the true edge.
    if known_closing_iterations > 0:
        known_grid = binary_closing(known_grid, iterations=known_closing_iterations)
    if known_dilate_cells > 0:
        known_grid = binary_dilation(known_grid, iterations=known_dilate_cells)

    free_grid = known_grid & ~occupied_grid
    if keep_largest_free:
        # Ghost/duplicate rooms from residual SLAM drift show up as small
        # disconnected free-space islands - only the region actually
        # reachable from the main mapped area is real.
        free_grid = keep_largest_component(free_grid)

    known_grid = free_grid | occupied_grid

    image = np.full(known_counts.shape, 205, dtype=np.uint8)  # unknown
    image[known_grid] = 254                                    # free
    image[occupied_grid] = 0                                   # occupied
    return image, origin_xy


def write_pgm(path: str, image: np.ndarray):
    # ROS map_server convention: row 0 of the file = maximum Y.
    flipped = np.flipud(image)
    height, width = flipped.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        f.write(flipped.tobytes())


def write_yaml(path: str, pgm_filename: str, resolution: float, origin_xy):
    with open(path, 'w') as f:
        f.write(f'image: {pgm_filename}\n')
        f.write(f'resolution: {resolution}\n')
        f.write(f'origin: [{origin_xy[0]}, {origin_xy[1]}, 0.0]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', required=True, help='Input PCD file path')
    parser.add_argument('--output-dir', default=os.path.expanduser('~/bebot_ws/src/bebot_navigation/maps'))
    parser.add_argument('--map-name', default='warehouse_map')
    parser.add_argument('--resolution', type=float, default=0.05, help='Meters per grid cell')
    parser.add_argument('--obstacle-min-height', type=float, default=0.05,
                         help='Meters above the estimated floor - lower bound of the obstacle band')
    parser.add_argument('--obstacle-max-height', type=float, default=0.6,
                         help='Meters above the estimated floor - upper bound of the obstacle band')
    parser.add_argument('--known-dilate-cells', type=int, default=1,
                         help='Final dilation of the "observed" region by this many cells')
    parser.add_argument('--known-closing-iterations', type=int, default=3,
                         help='Morphological closing passes on the observed mask (fills sampling-gap speckle inside it)')
    parser.add_argument('--min-known-points', type=int, default=1,
                         help='Minimum point hits in a cell to count it as observed at all')
    parser.add_argument('--min-occupied-points', type=int, default=1,
                         help='Minimum point hits within the obstacle band to mark a cell occupied')
    parser.add_argument('--opening-iterations', type=int, default=0,
                         help='Morphological opening passes on the occupied mask (off by default - see build_grid docstring)')
    parser.add_argument('--closing-iterations', type=int, default=1,
                         help='Morphological closing passes on the occupied mask (fuses/solidifies walls)')
    parser.add_argument('--min-blob-cells', type=int, default=5,
                         help='Drop occupied connected components smaller than this many cells')
    parser.add_argument('--no-largest-component', action='store_true',
                         help='Disable pruning free space down to the single largest connected region')
    args = parser.parse_args()

    print(f'Reading {args.pcd} ...')
    points = read_pcd_ascii(os.path.expanduser(args.pcd))
    print(f'{len(points)} points loaded.')

    floor_z = estimate_floor_z(points)
    print(f'Estimated floor Z: {floor_z:.3f} m')
    obstacle_z_range = (floor_z + args.obstacle_min_height, floor_z + args.obstacle_max_height)

    image, origin_xy = build_grid(
        points, args.resolution, obstacle_z_range,
        min_known_points=args.min_known_points,
        min_occupied_points=args.min_occupied_points,
        opening_iterations=args.opening_iterations,
        closing_iterations=args.closing_iterations,
        min_blob_cells=args.min_blob_cells,
        known_dilate_cells=args.known_dilate_cells,
        known_closing_iterations=args.known_closing_iterations,
        keep_largest_free=not args.no_largest_component,
    )
    print(f'Grid size: {image.shape[1]} x {image.shape[0]} cells at {args.resolution} m/cell')

    unknown, free, occupied = np.sum(image == 205), np.sum(image == 254), np.sum(image == 0)
    print(f'Cells: {free} free, {occupied} occupied, {unknown} unknown')

    os.makedirs(os.path.expanduser(args.output_dir), exist_ok=True)
    pgm_path = os.path.join(os.path.expanduser(args.output_dir), f'{args.map_name}.pgm')
    yaml_path = os.path.join(os.path.expanduser(args.output_dir), f'{args.map_name}.yaml')

    write_pgm(pgm_path, image)
    write_yaml(yaml_path, f'{args.map_name}.pgm', args.resolution, origin_xy)

    print(f'Wrote {pgm_path}')
    print(f'Wrote {yaml_path}')


if __name__ == '__main__':
    main()
