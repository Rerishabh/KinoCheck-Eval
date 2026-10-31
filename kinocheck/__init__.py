"""Geometry and validation utilities for KinoCheck-Eval."""

from .geometry import (
    create_obstacle_boxes,
    segment_min_distance,
    verify_polyline_clearance,
)

__all__ = [
    "create_obstacle_boxes",
    "segment_min_distance",
    "verify_polyline_clearance",
]
