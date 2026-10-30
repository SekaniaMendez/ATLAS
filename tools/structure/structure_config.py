"""Measured support limits for offline structural surface extraction."""
from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class StructureConfig:
    distance_threshold_m: float = 0.025
    normal_radius_m: float = 0.20
    normal_max_neighbors: int = 30
    normal_angle_degrees: float = 25.0
    ransac_iterations: int = 1200
    max_plane_hypotheses: int = 32
    min_support_points: int = 100
    min_area_m2: float = 0.12
    min_extent_m: float = 0.20
    connectivity_radius_m: float = 0.20
    mesh_sample_grid_m: float = 0.08
    mesh_max_edge_m: float = 0.25
    mesh_max_circumradius_m: float = 0.15
    contour_tolerance_m: float = 0.015
    orientation_angle_degrees: float = 12.0
    max_patches: int = 80
    random_seed: int = 7

    def validate(self):
        for name, value in asdict(self).items():
            if name == "random_seed":
                if type(value) is not int or not 0 <= value < 2**31:
                    raise ValueError("random_seed must be an integer in [0, 2**31)")
            elif name in {"normal_max_neighbors", "ransac_iterations",
                          "max_plane_hypotheses", "min_support_points", "max_patches"}:
                if type(value) is not int or value < 1:
                    raise ValueError(name + " must be a positive integer")
            elif not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")
        if self.min_support_points < 3 or self.normal_max_neighbors < 3:
            raise ValueError("plane fitting needs at least three points")
        if not 0 < self.normal_angle_degrees < 90:
            raise ValueError("normal_angle_degrees must be between 0 and 90")
        if not 0 < self.orientation_angle_degrees < 45:
            raise ValueError("orientation_angle_degrees must be between 0 and 45")
        if self.mesh_sample_grid_m > self.mesh_max_edge_m / 2:
            raise ValueError("mesh_sample_grid_m must not exceed half the maximum edge")
        return self
