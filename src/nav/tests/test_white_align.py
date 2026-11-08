from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

NAV_DIR = Path(__file__).resolve().parents[1]
if str(NAV_DIR) not in sys.path:
    sys.path.insert(0, str(NAV_DIR))

from white_align import (  # noqa: E402
    align_laterally,
    find_white_regions,
    lateral_offset,
    select_white_region,
)

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


def _image_with_rectangles(
    rectangles: list[tuple[int, int, int, int]],
) -> np.ndarray:
    image = np.zeros(
        (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
        dtype=np.uint8,
    )
    for left, top, right, bottom in rectangles:
        # OpenCV draws inclusive bounds; subtract one to keep the
        # bounding rectangle exactly (right - left) pixels wide.
        cv2.rectangle(
            image,
            (left, top),
            (right - 1, bottom - 1),
            (255, 255, 255),
            -1,
        )
    return image


def test_finds_white_label_on_black_table() -> None:
    image = _image_with_rectangles([(400, 200, 500, 260)])

    regions = find_white_regions(image)

    assert len(regions) == 1
    region = regions[0]
    assert region.center_x_px == pytest.approx(450.0)
    assert region.center_y_px == pytest.approx(230.0)
    assert lateral_offset(region, IMAGE_WIDTH) == pytest.approx(
        (450.0 - 320.0) / 320.0
    )
    assert lateral_offset(region, IMAGE_WIDTH) > 0.0


def test_ignores_gray_surface_and_small_specks() -> None:
    image = np.zeros(
        (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
        dtype=np.uint8,
    )
    cv2.rectangle(image, (10, 10), (200, 200), (120, 120, 120), -1)
    cv2.rectangle(image, (300, 300), (304, 304), (255, 255, 255), -1)

    assert find_white_regions(image) == []


def test_largest_white_region_is_first() -> None:
    image = _image_with_rectangles(
        [(50, 100, 150, 180), (300, 200, 480, 300)]
    )

    regions = find_white_regions(image)

    assert len(regions) == 2
    assert regions[0].area_px > regions[1].area_px
    assert regions[0].center_x_px == pytest.approx(390.0)


def test_selects_region_closest_to_previous_center() -> None:
    image = _image_with_rectangles(
        [(50, 100, 150, 180), (300, 200, 480, 300)]
    )
    regions = find_white_regions(image)

    largest = select_white_region(regions)
    tracked = select_white_region(
        regions,
        previous_center_x_px=100.0,
    )

    assert largest is not None
    assert tracked is not None
    assert largest.center_x_px == pytest.approx(390.0)
    assert tracked.center_x_px == pytest.approx(100.0)


def test_selects_region_closest_to_image_center_initially() -> None:
    image = _image_with_rectangles(
        [(20, 100, 260, 220), (300, 200, 380, 260)]
    )
    regions = find_white_regions(image)

    selected = select_white_region(regions, image_center_x_px=320.0)
    largest = select_white_region(regions)

    assert selected is not None
    assert selected.center_x_px == pytest.approx(340.0)
    assert largest is not None
    assert largest.center_x_px == pytest.approx(140.0)


class ScaledCamera:
    """Ideal camera whose target offset shrinks as the robot moves right."""

    def __init__(
        self,
        target_offset: float = 0.6,
        half_width_m: float = 0.5,
        mirrored: bool = False,
    ) -> None:
        self.robot_right_m = 0.0
        self.target_offset = target_offset
        self.half_width_m = half_width_m
        self.mirrored = mirrored
        self.moves: list[float] = []

    def capture(self) -> np.ndarray:
        scale = 1.0 if self.mirrored else -1.0
        offset = self.target_offset + scale * (
            self.robot_right_m / self.half_width_m
        )
        offset = max(-0.8, min(0.8, offset))
        center_x = IMAGE_WIDTH / 2.0 + offset * IMAGE_WIDTH / 2.0
        left = int(round(center_x - 20))
        right = int(round(center_x + 20))
        return _image_with_rectangles([(left, 200, right, 260)])

    def move_right(self, distance_m: float) -> None:
        self.moves.append(distance_m)
        self.robot_right_m += distance_m


def test_alignment_converges_and_moves_right() -> None:
    camera = ScaledCamera(target_offset=0.6)

    aligned = align_laterally(camera.capture, camera.move_right)

    assert aligned
    assert camera.robot_right_m == pytest.approx(0.3, abs=0.03)
    assert camera.moves
    assert all(move >= 0.0 for move in camera.moves)


def test_alignment_reverses_a_mirrored_image_axis() -> None:
    camera = ScaledCamera(target_offset=0.6, mirrored=True)

    aligned = align_laterally(camera.capture, camera.move_right)

    assert aligned
    assert camera.robot_right_m == pytest.approx(-0.3, abs=0.06)
    assert any(move < 0.0 for move in camera.moves)


class StuckCamera:
    """Camera that keeps reporting the same blob regardless of motion."""

    def __init__(self) -> None:
        self.moves: list[float] = []

    def capture(self) -> np.ndarray:
        return _image_with_rectangles([(400, 200, 500, 260)])

    def move_right(self, distance_m: float) -> None:
        self.moves.append(distance_m)


def test_alignment_stops_and_reverts_without_progress() -> None:
    camera = StuckCamera()

    aligned = align_laterally(camera.capture, camera.move_right)

    assert not aligned
    # Two corrections, then one move that undoes them.
    assert len(camera.moves) == 3
    assert sum(camera.moves) == pytest.approx(0.0, abs=1e-9)
    assert camera.moves[-1] < 0.0


def test_alignment_skips_without_camera_frame() -> None:
    moves: list[float] = []

    assert not align_laterally(lambda: None, moves.append)
    assert moves == []


def test_alignment_skips_without_white_label() -> None:
    moves: list[float] = []
    black = np.zeros(
        (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
        dtype=np.uint8,
    )

    assert not align_laterally(lambda: black, moves.append)
    assert moves == []


def test_lateral_offset_rejects_zero_width() -> None:
    image = _image_with_rectangles([(400, 200, 500, 260)])
    region = find_white_regions(image)[0]

    with pytest.raises(ValueError):
        lateral_offset(region, 0)
