#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read pickup codes printed on white parcel labels."""

from __future__ import annotations

import re
import shutil
import subprocess
from collections.abc import Callable
from typing import Any

try:
    from .white_align import WhiteRegion
except ImportError:  # pragma: no cover - direct script import
    from white_align import WhiteRegion

# Pickup codes have one letter followed by three digits. A leading 8 is
# a common OCR misread of B on these labels.
CODE_PATTERN = re.compile(r"[A-Z8][0-9]{3}")
DEFAULT_OCR_PSM_MODES = (7, 6, 11, 8, 13)
OCR_WHITELIST = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
DEFAULT_LANGUAGE = "eng"
DEFAULT_TIMEOUT_S = 5.0
LABEL_MARGIN_RATIO = 0.08


class LabelOcrError(RuntimeError):
    """Raised when the OCR engine is unavailable or fails."""


def normalize_label_code(text: str | None) -> str | None:
    """Extract a letter+3-digit code; a leading 8 is corrected to B."""
    if not text:
        return None
    compact = re.sub(r"[^A-Z0-9]", "", text.upper())
    match = CODE_PATTERN.search(compact)
    if match is None:
        return None
    code = match.group(0)
    if code[0] == "8":
        code = "B" + code[1:]
    return code


def _crop_label(
    image: Any,
    region: WhiteRegion,
    margin_ratio: float = LABEL_MARGIN_RATIO,
) -> Any | None:
    """Crop a white region with a small margin around its bounding box."""
    height, width = image.shape[:2]
    margin_x = int(round(region.width_px * margin_ratio))
    margin_y = int(round(region.height_px * margin_ratio))
    left = max(0, region.left_px - margin_x)
    top = max(0, region.top_px - margin_y)
    right = min(width, region.right_px + margin_x)
    bottom = min(height, region.bottom_px + margin_y)
    if right <= left or bottom <= top:
        return None
    return image[top:bottom, left:right]


def prepare_label_image(crop: Any, *, scale: int = 3) -> Any:
    """Convert a label crop to an upscaled, binarized OCR image."""
    import cv2

    if crop.ndim == 3:
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    else:
        gray = crop
    if scale > 1:
        gray = cv2.resize(
            gray,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_CUBIC,
        )
    _, binary = cv2.threshold(
        gray,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU,
    )
    border = max(10, int(round(min(binary.shape[:2]) * 0.1)))
    return cv2.copyMakeBorder(
        binary,
        border,
        border,
        border,
        border,
        cv2.BORDER_CONSTANT,
        value=255,
    )


def _run_tesseract(
    image: Any,
    executable: str,
    psm: int,
    *,
    language: str,
    timeout_s: float,
) -> str:
    import cv2

    encoded_ok, encoded = cv2.imencode(".png", image)
    if not encoded_ok:
        return ""
    command = [
        executable,
        "stdin",
        "stdout",
        "--psm",
        str(psm),
        "-l",
        language,
        "-c",
        f"tessedit_char_whitelist={OCR_WHITELIST}",
    ]
    try:
        completed = subprocess.run(
            command,
            input=encoded.tobytes(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise LabelOcrError(f"tesseract failed: {error}") from error
    if completed.returncode != 0:
        stderr = completed.stderr.decode(
            "utf-8",
            errors="replace",
        ).strip()
        raise LabelOcrError(
            f"tesseract exited with {completed.returncode}: {stderr}"
        )
    return completed.stdout.decode("utf-8", errors="replace")


def read_label_code(
    image: Any,
    region: WhiteRegion,
    *,
    executable: str | None = None,
    psm_modes: tuple[int, ...] = DEFAULT_OCR_PSM_MODES,
    language: str = DEFAULT_LANGUAGE,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    log: Callable[[str], None] = print,
) -> str | None:
    """OCR the white label region and return a normalized pickup code."""
    executable = executable or shutil.which("tesseract")
    if not executable:
        raise LabelOcrError(
            "tesseract executable not found; install tesseract-ocr to "
            "read parcel labels"
        )

    crop = _crop_label(image, region)
    if crop is None:
        log("[label-ocr] empty label crop")
        return None
    prepared = prepare_label_image(crop)
    for psm in psm_modes:
        text = _run_tesseract(
            prepared,
            executable,
            psm,
            language=language,
            timeout_s=timeout_s,
        )
        code = normalize_label_code(text)
        if code is not None:
            log(f"[label-ocr] psm {psm}: {text.strip()!r} -> {code}")
            return code
        log(f"[label-ocr] psm {psm}: no valid code in {text.strip()!r}")
    return None
