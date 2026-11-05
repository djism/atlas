# Changelog

## Unreleased

- Added `computeCentroid()` for nonempty 2D point sets.
- Added `estimateFromMatchedPoints()`: least-squares rigid alignment from two or
  more known pairs, with scale-aware rejection of coincident or ambiguous input.
- Added `computeMatchedRmse()` for known-pair alignment error.
- Demo now uses the general estimator; `estimateFromTwoPairs()` remains as a baseline.
- Tests cover centroids, multi-point recovery, angle wrapping, reordering,
  collinear input, invalid input, and a fixed noisy least-squares example.
- Documented the derivation and limitations in `docs/alignment.md`.

## 0.1.0 — 2026-11-04

- Added C++17 2D point and pose types with explicit frame conventions.
- Added forward/inverse rigid transforms and ideal range-limited scan simulation.
- Added nearest-point search and a two-known-pair relative pose estimator.
- Added deterministic demo, numerical checks, CMake build, and GitHub Actions CI.
- Documented scope, daily milestones, and planned progression toward SLAM.
