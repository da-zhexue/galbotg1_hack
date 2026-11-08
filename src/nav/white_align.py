#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""White-label detection and lateral alignment for pickup tables."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

# Tuned for a white parcel label on a black table under indoor light.
DEFAULT_SATURATION_MAX = 80
DEFAULT_VALUE_MIN = 140
DEFAULT_MIN_AREA_RATIO = 0.001
DEFAULT_MAX_AREA_RATIO = 0.5
DEFAULT_CLOSE_KERNEL_PX = 5

ALIGN_DEADBAND = 0.04
ALIGN_GAIN_M = 0.30
ALIGN_MAX_STEP_M = 0.15
ALIGN_MAX_ITERATIONS = 7
# A correction must reduce |offset| by at least this much to count as
# progress; two corrections in a row without progress abort the alignment.
ALIGN_PROGRESS_EPS = 0.01
ALIGN_MAX_STALLS = 2
ALIGN_MAX_REVERSALS = 1
# Never let the calibration walk more than this far from the start.
ALIGN_MAX_TRAVEL_M = 0.30


@dataclass(frozen=True)
class WhiteRegion:
    """Bounding box of one white blob in a BGR image."""

    left_px: int
    top_px: int
    right_px: int
    bottom_px: int
    area_px: float

    @property
    def width_px(self) -> int:
        return self.right_px - self.left_px

    @property
    def height_px(self) -> int:
        return self.bottom_px - self.top_px

    @property
    def center_x_px(self) -> float:
        return (self.left_px + self.right_px) / 2.0

    @property
    def center_y_px(self) -> float:
        return (self.top_px + self.bottom_px) / 2.0


def find_white_regions(
    image: Any,
    *,
    saturation_max: int = DEFAULT_SATURATION_MAX,
    value_min: int = DEFAULT_VALUE_MIN,
    min_area_ratio: float = DEFAULT_MIN_AREA_RATIO,
    max_area_ratio: float = DEFAULT_MAX_AREA_RATIO,
    close_kernel_px: int = DEFAULT_CLOSE_KERNEL_PX,
) -> list[WhiteRegion]:
    """Return white blobs of a BGR image, largest area first."""
    import cv2
    import numpy as np

    if image is None or getattr(image, "size", 0) == 0:
        return []
    if image.ndim == 2:
        bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    elif image.shape[2] == 4:
        bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    else:
        bgr = image

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        (0, 0, int(value_min)),
        (179, int(saturation_max), 255),
    )
    if close_kernel_px >= 3:
        kernel = np.ones(
            (int(close_kernel_px), int(close_kernel_px)),
            dtype=np.uint8,
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    image_area = float(image.shape[0] * image.shape[1])
    regions: list[WhiteRegion] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        ratio = area / image_area
        if ratio < min_area_ratio or ratio > max_area_ratio:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        regions.append(
            WhiteRegion(
                left_px=int(x),
                top_px=int(y),
                right_px=int(x + width),
                bottom_px=int(y + height),
                area_px=area,
            )
        )
    regions.sort(key=lambda region: region.area_px, reverse=True)
    return regions


def lateral_offset(region: WhiteRegion, image_width: int) -> float:
    """Return the horizontal offset in [-1, 1]; positive is image right."""
    if image_width <= 0:
        raise ValueError("image_width must be positive")
    half_width = image_width / 2.0
    return (region.center_x_px - half_width) / half_width


def select_white_region(
    regions: Sequence[WhiteRegion],
    previous_center_x_px: float | None = None,
    *,
    image_center_x_px: float | None = None,
) -> WhiteRegion | None:
    """Pick the blob nearest a reference point.

    With a previous center, the nearest blob is tracked across frames.
    Without history, the blob nearest the image center is chosen so a
    large white background object cannot win by area alone. If no image
    center is given either, the largest blob is returned.
    """
    if not regions:
        return None
    if previous_center_x_px is not None:
        reference_x = previous_center_x_px
    elif image_center_x_px is not None:
        reference_x = image_center_x_px
    else:
        return max(regions, key=lambda region: region.area_px)
    return min(
        regions,
        key=lambda region: abs(region.center_x_px - reference_x),
    )


def align_laterally(
    capture_image: Callable[[], Any | None],
    move_right: Callable[[float], None],
    detect: Callable[..., list[WhiteRegion]] = find_white_regions,
    *,
    deadband: float = ALIGN_DEADBAND,
    gain_m: float = ALIGN_GAIN_M,
    max_step_m: float = ALIGN_MAX_STEP_M,
    max_iterations: int = ALIGN_MAX_ITERATIONS,
    max_travel_m: float = ALIGN_MAX_TRAVEL_M,
    log: Callable[[str], None] = print,
) -> bool:
    """Center the detected white label by moving the base left/right.

    ``move_right(distance_m)`` must move the base to the robot's right for
    a positive distance. The loop is closed-loop, so ``gain_m`` only sets
    the convergence speed; the image axis direction is verified while
    iterating and reversed when a correction step made things worse.

    The alignment gives up instead of walking away when the measured
    offset stops improving, and then moves back to the starting lateral
    position. This guards against locking onto a different white object
    (or a white part of the robot) mid-correction.
    """
    if deadband <= 0.0:
        raise ValueError("deadband must be positive")
    if gain_m <= 0.0:
        raise ValueError("gain_m must be positive")
    if max_step_m <= 0.0:
        raise ValueError("max_step_m must be positive")
    if max_iterations <= 0:
        raise ValueError("max_iterations must be positive")
    if max_travel_m <= 0.0:
        raise ValueError("max_travel_m must be positive")

    previous_center_x: float | None = None
    previous_abs_offset: float | None = None
    direction = 1.0
    reversals = 0
    stalls = 0
    net_step = 0.0

    def revert(reason: str) -> None:
        if abs(net_step) <= 1e-9:
            log(
                f"[white-align] {reason}; "
                "keeping the current lateral position"
            )
            return
        log(
            f"[white-align] {reason}; moving {-net_step:+.3f} m "
            "back to the starting lateral position"
        )
        move_right(-net_step)

    for iteration in range(1, max_iterations + 1):
        image = capture_image()
        if image is None:
            revert("camera frame unavailable")
            return False

        image_width = int(image.shape[1])
        regions = detect(image)
        region = select_white_region(
            regions,
            previous_center_x,
            image_center_x_px=image_width / 2.0,
        )
        if region is None:
            revert("no white label found")
            return False

        offset = lateral_offset(region, image_width)
        log(
            f"[white-align] try {iteration}/{max_iterations}: "
            f"white center x={region.center_x_px:.1f}px "
            f"area={region.area_px:.0f}, offset={offset:+.3f}"
        )
        if abs(offset) <= deadband:
            log("[white-align] laterally aligned")
            return True

        if previous_abs_offset is not None:
            improvement = previous_abs_offset - abs(offset)
            if improvement < -ALIGN_PROGRESS_EPS:
                if reversals >= ALIGN_MAX_REVERSALS:
                    revert("offset kept growing after reversing")
                    return False
                direction = -direction
                reversals += 1
                stalls = 0
                log(
                    "[white-align] offset grew after the last move; "
                    "reversing the correction direction"
                )
            elif improvement < ALIGN_PROGRESS_EPS:
                stalls += 1
                log(
                    "[white-align] offset did not improve "
                    f"({stalls}/{ALIGN_MAX_STALLS})"
                )
                if stalls >= ALIGN_MAX_STALLS:
                    revert("offset stopped improving")
                    return False
            else:
                stalls = 0

        step_right = direction * gain_m * offset
        step_right = max(-max_step_m, min(max_step_m, step_right))
        if abs(net_step + step_right) > max_travel_m:
            revert("correction travel limit reached")
            return False

        log(f"[white-align] moving {step_right:+.3f} m to the right")
        move_right(step_right)
        net_step += step_right
        previous_abs_offset = abs(offset)
        previous_center_x = region.center_x_px

    revert("calibration did not converge")
    return False
