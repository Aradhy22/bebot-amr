"""Minimal point-to-point ICP (no Open3D/PCL-python dependency needed).

Standard iterative closest point: nearest-neighbour correspondence via a
KD-tree, optimal rigid transform via SVD (Kabsch/Procrustes), repeat until
convergence or max iterations. Good enough for the small (~1-5k point),
already-roughly-aligned keyframe clouds used for loop closure here - this
is not meant to compete with a real GICP/NDT implementation.
"""

import numpy as np
from scipy.spatial import cKDTree


def best_fit_transform(source: np.ndarray, target: np.ndarray):
    """Optimal rotation+translation aligning source onto target (Nx3 each,
    same length, already-corresponded). Returns (R, t) with target ~= R@source + t.
    """
    src_mean = source.mean(axis=0)
    tgt_mean = target.mean(axis=0)
    src_c = source - src_mean
    tgt_c = target - tgt_mean

    H = src_c.T @ tgt_c
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = tgt_mean - R @ src_mean
    return R, t


def icp(source: np.ndarray, target: np.ndarray, init_R: np.ndarray, init_t: np.ndarray,
        max_iterations: int = 30, max_correspondence_dist: float = 1.0,
        tolerance: float = 1e-5):
    """Align source onto target, starting from an initial guess transform.

    Returns (R, t, fitness) where fitness is the mean residual distance of
    the final inlier correspondences (lower is better; np.inf if too few
    correspondences survived the distance gate to trust the result).
    """
    R, t = init_R.copy(), init_t.copy()
    target_tree = cKDTree(target)

    prev_error = None
    inlier_ratio = 0.0
    mean_residual = np.inf

    for _ in range(max_iterations):
        transformed = (R @ source.T).T + t

        dists, idx = target_tree.query(transformed, k=1)
        mask = dists < max_correspondence_dist
        if mask.sum() < 10:
            return R, t, np.inf, 0.0

        matched_src = source[mask]
        matched_tgt = target[idx[mask]]

        R, t = best_fit_transform(matched_src, matched_tgt)

        mean_residual = float(dists[mask].mean())
        inlier_ratio = float(mask.sum()) / len(source)

        if prev_error is not None and abs(prev_error - mean_residual) < tolerance:
            break
        prev_error = mean_residual

    return R, t, mean_residual, inlier_ratio
