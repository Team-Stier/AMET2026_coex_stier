from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    horizontal_fov_rad: float
    distortion: tuple[float, float, float, float, float]

    @property
    def matrix(self) -> np.ndarray:
        focal = self.width / (2.0 * np.tan(self.horizontal_fov_rad / 2.0))
        return np.array(
            [
                [focal, 0.0, (self.width - 1.0) / 2.0],
                [0.0, focal, (self.height - 1.0) / 2.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )


@dataclass(frozen=True)
class BevGeometry:
    x_min_m: float = 0.15
    x_max_m: float = 3.0
    y_left_m: float = 1.2
    y_right_m: float = 1.2
    resolution_m: float = 0.02
    ground_z_m: float = 0.0

    @property
    def height(self) -> int:
        return int(round((self.x_max_m - self.x_min_m) / self.resolution_m)) + 1

    @property
    def width(self) -> int:
        return int(round((self.y_left_m + self.y_right_m) / self.resolution_m)) + 1

    def ground_grids(self) -> tuple[np.ndarray, np.ndarray]:
        x_values = np.linspace(self.x_max_m, self.x_min_m, self.height)
        y_values = np.linspace(self.y_left_m, -self.y_right_m, self.width)
        return np.meshgrid(x_values, y_values, indexing="ij")

    def pixel_to_ground(
        self, rows: np.ndarray, columns: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        x_m = self.x_max_m - rows * self.resolution_m
        y_m = self.y_left_m - columns * self.resolution_m
        return x_m, y_m

    def ground_to_pixel(
        self, x_m: np.ndarray, y_m: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        rows = (self.x_max_m - x_m) / self.resolution_m
        columns = (self.y_left_m - y_m) / self.resolution_m
        return rows, columns


def transform_to_matrix(transform) -> np.ndarray:
    translation = transform.translation
    rotation = transform.rotation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("camera transform has a zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


class BevProjector:
    def __init__(self, camera: CameraModel, geometry: BevGeometry):
        self.camera = camera
        self.geometry = geometry

    def project(self, image: np.ndarray, camera_from_base: np.ndarray) -> np.ndarray:
        if image.shape[:2] != (self.camera.height, self.camera.width):
            raise ValueError(
                "camera image size does not match the configured model: "
                f"{image.shape[1]}x{image.shape[0]} != "
                f"{self.camera.width}x{self.camera.height}"
            )
        if camera_from_base.shape != (4, 4):
            raise ValueError("camera_from_base must be a 4x4 matrix")

        x_grid, y_grid = self.geometry.ground_grids()
        point_count = x_grid.size
        base_points = np.vstack(
            (
                x_grid.reshape(-1),
                y_grid.reshape(-1),
                np.full(point_count, self.geometry.ground_z_m),
                np.ones(point_count),
            )
        )
        camera_points = camera_from_base @ base_points
        x_cam, y_cam, z_cam = camera_points[:3]

        valid = z_cam > 1e-6
        normalized_x = np.zeros_like(z_cam)
        normalized_y = np.zeros_like(z_cam)
        normalized_x[valid] = x_cam[valid] / z_cam[valid]
        normalized_y[valid] = y_cam[valid] / z_cam[valid]

        k1, k2, p1, p2, k3 = self.camera.distortion
        radius_sq = normalized_x * normalized_x + normalized_y * normalized_y
        radial = 1.0 + k1 * radius_sq + k2 * radius_sq**2 + k3 * radius_sq**3
        distorted_x = (
            normalized_x * radial
            + 2.0 * p1 * normalized_x * normalized_y
            + p2 * (radius_sq + 2.0 * normalized_x * normalized_x)
        )
        distorted_y = (
            normalized_y * radial
            + p1 * (radius_sq + 2.0 * normalized_y * normalized_y)
            + 2.0 * p2 * normalized_x * normalized_y
        )

        camera_matrix = self.camera.matrix
        map_x = camera_matrix[0, 0] * distorted_x + camera_matrix[0, 2]
        map_y = camera_matrix[1, 1] * distorted_y + camera_matrix[1, 2]
        map_x[~valid] = -1.0
        map_y[~valid] = -1.0

        return cv2.remap(
            image,
            map_x.reshape(self.geometry.height, self.geometry.width).astype(np.float32),
            map_y.reshape(self.geometry.height, self.geometry.width).astype(np.float32),
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
