from __future__ import annotations

import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

NAV_DIR = Path(__file__).resolve().parents[1]
if str(NAV_DIR) not in sys.path:
    sys.path.insert(0, str(NAV_DIR))

from label_ocr import (  # noqa: E402
    LabelOcrError,
    normalize_label_code,
    read_label_code,
)
from white_align import WhiteRegion  # noqa: E402


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("B520", "B520"),
        ("b 5 2 0", "B520"),
        ("B-520", "B520"),
        ("8520", "B520"),
        ("8512", "B512"),
        ("code: B520", "B520"),
        ("B5200", "B520"),
        ("A123", "A123"),
    ],
)
def test_normalize_label_code(text: str, expected: str) -> None:
    assert normalize_label_code(text) == expected


@pytest.mark.parametrize(
    "text",
    ["", "B52", "BS20", "1234", "ABC", None],
)
def test_normalize_label_code_rejects_invalid(
    text: str | None,
) -> None:
    assert normalize_label_code(text) is None


def _label_image() -> tuple[np.ndarray, WhiteRegion]:
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (200, 150), (440, 290), (255, 255, 255), -1)
    cv2.putText(
        image,
        "B520",
        (240, 245),
        cv2.FONT_HERSHEY_SIMPLEX,
        2.0,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    region = WhiteRegion(
        left_px=200,
        top_px=150,
        right_px=440,
        bottom_px=290,
        area_px=240.0 * 140.0,
    )
    return image, region


@pytest.mark.skipif(
    shutil.which("tesseract") is None,
    reason="tesseract is not installed",
)
def test_reads_printed_code_from_white_label() -> None:
    image, region = _label_image()

    code = read_label_code(image, region, log=lambda _: None)

    assert code == "B520"


def test_reports_missing_tesseract() -> None:
    image, region = _label_image()

    with pytest.raises(LabelOcrError):
        read_label_code(
            image,
            region,
            executable="/nonexistent/tesseract",
            log=lambda _: None,
        )
