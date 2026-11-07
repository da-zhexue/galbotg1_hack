#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PCM voice prompt playback for a Galbot G1."""

from __future__ import annotations

import time
from collections.abc import Callable
from pathlib import Path

from galbot_sdk.g1 import GalbotRobot

PCM_SAMPLE_RATE_HZ = 16000
PCM_SAMPLE_WIDTH_BYTES = 2
PCM_CHANNELS = 1
PCM_BYTES_PER_SECOND = (
    PCM_SAMPLE_RATE_HZ * PCM_SAMPLE_WIDTH_BYTES * PCM_CHANNELS
)
DEFAULT_CHUNK_BYTES = 2560  # 80 ms of 16 kHz, 16-bit mono PCM

WELCOME = 1
PICKUP_ANNOUNCEMENT = 2
ORDER_SUCCESS = 3
PICKED = 4
ARRIVING = 5
DELIVERED = 6

AUDIO_TRACKS = {
    WELCOME: "01_welcome.pcm",
    PICKUP_ANNOUNCEMENT: "02_galbot_pickup_announcement.pcm",
    ORDER_SUCCESS: "03_order_success.pcm",
    PICKED: "04_picked.pcm",
    ARRIVING: "05_arriving.pcm",
    DELIVERED: "06_delivered.pcm",
}

DEFAULT_AUDIO_DIR = Path(__file__).resolve().parents[2] / "mp3"


class VoiceError(RuntimeError):
    """Raised when an audio file or playback command fails."""


class G1Voice:
    """Play the numbered PCM prompts through the official SDK."""

    def __init__(
        self,
        robot: GalbotRobot,
        *,
        audio_dir: str | Path | None = None,
        stream_id: str = "voice",
        chunk_bytes: int = DEFAULT_CHUNK_BYTES,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if robot is None:
            raise ValueError("robot must be a GalbotRobot instance")
        if chunk_bytes <= 0:
            raise ValueError("chunk_bytes must be positive")
        self.robot = robot
        self.audio_dir = (
            Path(audio_dir) if audio_dir is not None else DEFAULT_AUDIO_DIR
        )
        self.stream_id = stream_id
        self.chunk_bytes = int(chunk_bytes)
        self.sleep = sleep

    def track_path(self, track: int) -> Path:
        """Return the PCM file for a numbered prompt."""
        try:
            filename = AUDIO_TRACKS[track]
        except (KeyError, TypeError) as error:
            raise ValueError(f"unknown voice track: {track!r}") from error
        return self.audio_dir / filename

    def play(self, track: int) -> None:
        """Stream one PCM prompt and block until it finishes playing."""
        path = self.track_path(track)
        try:
            data = path.read_bytes()
        except OSError as error:
            raise VoiceError(
                f"failed to read audio file {path}: {error}"
            ) from error
        if not data:
            raise VoiceError(f"audio file is empty: {path}")

        start = time.monotonic()
        written = 0
        for offset in range(0, len(data), self.chunk_bytes):
            chunk = data[offset:offset + self.chunk_bytes]
            if not self.robot.write_audio_stream_output(
                chunk,
                self.stream_id,
            ):
                raise VoiceError(
                    "failed to write audio chunk at byte "
                    f"{offset} for track {track}"
                )
            written += len(chunk)
            target_s = written / PCM_BYTES_PER_SECOND
            remaining_s = target_s - (time.monotonic() - start)
            if remaining_s > 0.0:
                self.sleep(remaining_s)

    def stop(self) -> None:
        """Stop any prompt still playing on this stream."""
        self.robot.stop_audio_stream_output(self.stream_id)
