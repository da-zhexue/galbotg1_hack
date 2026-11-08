#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Galbot G1 pickup and delivery state machine."""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parents[1]
for import_dir in (SRC_DIR, SRC_DIR / "grabbox"):
    if str(import_dir) not in sys.path:
        sys.path.insert(0, str(import_dir))

from galbot_sdk.g1 import GalbotRobot
from grabbox import GrabBox
from nav.g1_route import G1Route
from voice.g1_voice import (
    ARRIVING,
    DELIVERED,
    G1Voice,
    ORDER_SUCCESS,
    PICKED,
    PICKUP_ANNOUNCEMENT,
)


class DeliveryError(RuntimeError):
    """Raised when the delivery workflow cannot continue."""


class PickupCodeMismatch(DeliveryError):
    """Raised when no scan position finds the requested pickup code."""


class DeliveryState(str, Enum):
    IDLE = "idle"
    ANNOUNCING = "announcing"
    APPROACHING_TABLE = "approaching_table"
    SCANNING_1 = "scanning_1"
    MOVING_TO_SCAN_2 = "moving_to_scan_2"
    SCANNING_2 = "scanning_2"
    MOVING_TO_SCAN_3 = "moving_to_scan_3"
    SCANNING_3 = "scanning_3"
    GRABBING = "grabbing"
    LEAVING_TABLE = "leaving_table"
    NAVIGATING_TO_DOOR = "navigating_to_door"
    DROPPING = "dropping"
    RESETTING = "resetting"
    RETURNING_TO_ORIGIN = "returning_to_origin"
    COMPLETED = "completed"
    FAILED = "failed"


TERMINAL_STATES = frozenset(
    {DeliveryState.COMPLETED, DeliveryState.FAILED}
)

TRANSITIONS: dict[DeliveryState, frozenset[DeliveryState]] = {
    DeliveryState.IDLE: frozenset({DeliveryState.ANNOUNCING}),
    DeliveryState.ANNOUNCING: frozenset(
        {DeliveryState.APPROACHING_TABLE}
    ),
    DeliveryState.APPROACHING_TABLE: frozenset(
        {DeliveryState.SCANNING_1}
    ),
    DeliveryState.SCANNING_1: frozenset(
        {
            DeliveryState.MOVING_TO_SCAN_2,
            DeliveryState.GRABBING,
        }
    ),
    DeliveryState.MOVING_TO_SCAN_2: frozenset(
        {DeliveryState.SCANNING_2}
    ),
    DeliveryState.SCANNING_2: frozenset(
        {
            DeliveryState.MOVING_TO_SCAN_3,
            DeliveryState.GRABBING,
        }
    ),
    DeliveryState.MOVING_TO_SCAN_3: frozenset(
        {DeliveryState.SCANNING_3}
    ),
    DeliveryState.SCANNING_3: frozenset({DeliveryState.GRABBING}),
    DeliveryState.GRABBING: frozenset(
        {DeliveryState.LEAVING_TABLE}
    ),
    DeliveryState.LEAVING_TABLE: frozenset(
        {DeliveryState.NAVIGATING_TO_DOOR}
    ),
    DeliveryState.NAVIGATING_TO_DOOR: frozenset(
        {DeliveryState.DROPPING}
    ),
    DeliveryState.DROPPING: frozenset({DeliveryState.RESETTING}),
    DeliveryState.RESETTING: frozenset(
        {DeliveryState.RETURNING_TO_ORIGIN}
    ),
    DeliveryState.RETURNING_TO_ORIGIN: frozenset(
        {DeliveryState.COMPLETED}
    ),
    DeliveryState.COMPLETED: frozenset(),
    DeliveryState.FAILED: frozenset(),
}


@dataclass(frozen=True)
class PickupCommand:
    pickup_code: str
    destination: str = "住户门口"


class DeliveryStateMachine:
    """Execute the pickup flow defined in 流程.md."""

    SCAN_POSITIONS = 3
    # Right-hand offsets of the three scan positions from the first one:
    # first parcel -> second parcel 0.50 m, second -> third 0.55 m.
    SCAN_RIGHT_OFFSETS_M = (0.0, 0.50, 1.05)
    # "Outside the table" is 1.00 m to the right of the third parcel.
    TABLE_EXIT_RIGHT_M = 1.05 + 1.00

    def __init__(
        self,
        robot: GalbotRobot,
        route: G1Route,
        grabber: GrabBox,
        voice: G1Voice,
        *,
        scan_codes: Sequence[str] | None = None,
        log: Callable[[str], None] = print,
    ) -> None:
        self.robot = robot
        self.route = route
        self.grabber = grabber
        self.voice = voice
        self.scan_codes = (
            tuple(scan_codes) if scan_codes is not None else None
        )
        self.log = log
        self.state = DeliveryState.IDLE
        self.command: PickupCommand | None = None
        self.scan_position: int | None = None
        self.last_scan_code: str | None = None
        self.last_error: str | None = None

    def start(self, command: PickupCommand) -> DeliveryState:
        if self.state != DeliveryState.IDLE:
            raise DeliveryError(
                f"state machine already started: {self.state.value}"
            )
        code = command.pickup_code.strip()
        destination = command.destination.strip()
        if not code:
            raise ValueError("pickup_code must not be empty")
        if not destination:
            raise ValueError("destination must not be empty")

        self.command = PickupCommand(code, destination)
        self.scan_position = None
        self.last_scan_code = None
        self.last_error = None
        self._transition(DeliveryState.ANNOUNCING, "UI command received")
        return self.run()

    def run(self) -> DeliveryState:
        if self.command is None:
            raise DeliveryError("pickup command has not been received")

        while self.state not in TERMINAL_STATES:
            try:
                self.step()
            except KeyboardInterrupt:
                raise
            except Exception as error:
                self._fail(error)
        return self.state

    def step(self) -> DeliveryState:
        handler = self._handlers().get(self.state)
        if handler is None:
            raise DeliveryError(
                f"no handler for state {self.state.value}"
            )
        handler()
        return self.state

    def _handlers(self) -> dict[DeliveryState, Callable[[], None]]:
        return {
            DeliveryState.ANNOUNCING: self._announce_command,
            DeliveryState.APPROACHING_TABLE: self._approach_table,
            DeliveryState.SCANNING_1: self._scan_1,
            DeliveryState.MOVING_TO_SCAN_2: self._move_to_scan_2,
            DeliveryState.SCANNING_2: self._scan_2,
            DeliveryState.MOVING_TO_SCAN_3: self._move_to_scan_3,
            DeliveryState.SCANNING_3: self._scan_3,
            DeliveryState.GRABBING: self._grab,
            DeliveryState.LEAVING_TABLE: self._leave_table,
            DeliveryState.NAVIGATING_TO_DOOR: self._navigate_to_door,
            DeliveryState.DROPPING: self._drop,
            DeliveryState.RESETTING: self._reset,
            DeliveryState.RETURNING_TO_ORIGIN: self._return_to_origin,
        }

    def _transition(
        self,
        target: DeliveryState,
        reason: str,
    ) -> None:
        if target == DeliveryState.FAILED:
            if self.state in TERMINAL_STATES:
                return
        elif target not in TRANSITIONS[self.state]:
            raise DeliveryError(
                f"invalid transition {self.state.value} -> "
                f"{target.value}"
            )

        previous = self.state
        self.state = target
        self.log(
            f"[state] {previous.value} -> {target.value}: {reason}"
        )

    def _fail(self, error: Exception) -> None:
        self.last_error = f"{type(error).__name__}: {error}"
        self.log(f"[state] workflow failed: {self.last_error}")

        try:
            self.robot.stop_trajectory_execution()
        except Exception as stop_error:
            self.log(f"[state] trajectory stop failed: {stop_error}")

        try:
            self.route.stop_base()
        except Exception as stop_error:
            self.log(f"[state] base stop failed: {stop_error}")

        self._speak("任务失败，已停止运动")
        self._transition(DeliveryState.FAILED, str(error))

    def _announce_command(self) -> None:
        command = self._command()
        self._speak(f"收到指令，前往取件{command.pickup_code}")
        self.voice.play(PICKUP_ANNOUNCEMENT)
        self._transition(
            DeliveryState.APPROACHING_TABLE,
            "command announced",
        )

    def _approach_table(self) -> None:
        self._prepare_motion()
        self.route.forward_60cm()
        self._transition(
            DeliveryState.SCANNING_1,
            "reached the first scan position",
        )

    def _scan_1(self) -> None:
        self._scan(1, DeliveryState.MOVING_TO_SCAN_2)

    def _move_to_scan_2(self) -> None:
        self._prepare_motion()
        self.route.right_50cm()
        self._transition(
            DeliveryState.SCANNING_2,
            "reached second scan position",
        )

    def _scan_2(self) -> None:
        self._scan(2, DeliveryState.MOVING_TO_SCAN_3)

    def _move_to_scan_3(self) -> None:
        self._prepare_motion()
        self.route.right_55cm()
        self._transition(
            DeliveryState.SCANNING_3,
            "reached third scan position",
        )

    def _scan_3(self) -> None:
        self._scan(3, None)

    def _scan(
        self,
        position: int,
        mismatch_next: DeliveryState | None,
    ) -> None:
        self.scan_position = position
        code = self._read_pickup_code(position)
        self.last_scan_code = code
        if position == self.SCAN_POSITIONS:
            self.voice.play(ORDER_SUCCESS)
        command = self._command()

        if code == command.pickup_code:
            self._speak(f"取件码{code}匹配，进行取件")
            self._transition(
                DeliveryState.GRABBING,
                f"pickup code matched at scan {position}",
            )
            return

        spoken_code = code if code else "未识别"
        self._speak(f"取件码{spoken_code}，不符")
        if mismatch_next is None:
            raise PickupCodeMismatch(
                f"pickup code {code!r} did not match "
                f"{command.pickup_code!r} after "
                f"{self.SCAN_POSITIONS} scans"
            )
        self._transition(
            mismatch_next,
            f"pickup code mismatch at scan {position}",
        )

    def _read_pickup_code(self, position: int) -> str | None:
        # TODO: 调用真实扫码模块，替换下面的演示结果。
        if self.scan_codes is not None:
            index = position - 1
            if 0 <= index < len(self.scan_codes):
                return self.scan_codes[index]
            return None

        command = self._command()
        mismatch_code = (
            "A-2345"
            if command.pickup_code != "A-2345"
            else "A-0000"
        )
        if position < self.SCAN_POSITIONS:
            return mismatch_code
        return command.pickup_code

    def _speak(self, text: str) -> None:
        # Dynamic text is logged only; fixed prompts go through G1Voice.
        print(f"[voice] {text}", flush=True)

    def _check_path_clear(self) -> None:
        # TODO: 接入激光雷达/视觉障碍检测，发现障碍时抛出异常。
        pass

    def _grab(self) -> None:
        self._prepare_motion()
        self.route.forward_40cm()
        self.route.check_stationary()
        self.grabber.grab()
        self.grabber.lift()
        self.voice.play(PICKED)
        self._transition(
            DeliveryState.LEAVING_TABLE,
            "box grasped",
        )

    def _leave_table(self) -> None:
        self._prepare_motion()
        if self.scan_position is None:
            raise DeliveryError("scan position is not recorded")
        right_offset = self.SCAN_RIGHT_OFFSETS_M[self.scan_position - 1]
        distance = self.TABLE_EXIT_RIGHT_M - right_offset
        self.route.backward_40cm()
        self.route.leave_table(distance)
        self._transition(
            DeliveryState.NAVIGATING_TO_DOOR,
            "cleared the table edge",
        )

    def _navigate_to_door(self) -> None:
        self._prepare_motion()
        self._speak(f"正在前往{self._command().destination}")
        self.route.forward_200cm()
        self.route.fixed_point_1()
        # self.route.fixed_point_2()
        # self.route.fixed_point_3()
        self.voice.play(ARRIVING)
        self._transition(
            DeliveryState.DROPPING,
            "arrived at destination",
        )

    def _drop(self) -> None:
        self.route.check_stationary()
        self.grabber.drop()
        self.voice.play(DELIVERED)
        self._transition(
            DeliveryState.RESETTING,
            "box released",
        )

    def _reset(self) -> None:
        self.grabber.default()
        self._speak("任务完成，正在返回原点")
        self._transition(
            DeliveryState.RETURNING_TO_ORIGIN,
            "default state restored",
        )

    def _return_to_origin(self) -> None:
        self._prepare_motion()
        self.route.return_to_origin()
        self._speak("任务完成，已回到原点")
        self._transition(
            DeliveryState.COMPLETED,
            "returned to origin",
        )

    def _prepare_motion(self) -> None:
        self._check_path_clear()
        self.route.check_stationary()

    def _command(self) -> PickupCommand:
        if self.command is None:
            raise DeliveryError("pickup command has not been received")
        return self.command


def _parse_scan_codes(value: str) -> tuple[str, ...]:
    codes = tuple(
        code.strip() for code in value.split(",") if code.strip()
    )
    if not codes:
        raise argparse.ArgumentTypeError(
            "scan codes must not be empty"
        )
    return codes


def _parse_args(
    argv: Sequence[str] | None = None,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Execute the pickup and delivery flow on a real Galbot G1."
        )
    )
    parser.add_argument(
        "--pickup-code",
        default="A-1234",
        help="Pickup code supplied by the UI command.",
    )
    parser.add_argument(
        "--destination",
        default="住户门口",
        help="Delivery destination.",
    )
    parser.add_argument(
        "--scan-codes",
        type=_parse_scan_codes,
        default=None,
        help="Temporary scan results for demo and testing.",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=25.0,
        help="Timeout for each chassis pose command.",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.5,
        help="Pause after each movement in seconds.",
    )
    parser.add_argument(
        "--controller-settle-s",
        type=float,
        default=1.0,
        help="Pause after switching the chassis controller.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the interactive safety confirmation.",
    )
    return parser.parse_args(argv)


def _confirm_safety() -> bool:
    print("WARNING:")
    print("  1. Release the emergency stop button.")
    print("  2. Verify the robot is in WORKING_MODE.")
    print("  3. Clear people and obstacles within 2 m.")
    print("  4. Keep the emergency stop within reach.")
    answer = input("Type YES to start the delivery flow: ").strip()
    return answer == "YES"


def _stop_voice(voice: G1Voice | None) -> None:
    if voice is None:
        return
    try:
        voice.stop()
    except Exception as error:
        print(f"[core] voice stop error: {error}")


def _stop_motion(
    robot: GalbotRobot,
    route: G1Route | None,
) -> None:
    try:
        robot.stop_trajectory_execution()
    except Exception as error:
        print(f"[core] trajectory stop error: {error}")

    if route is not None:
        try:
            route.stop_base()
            return
        except Exception as error:
            print(f"[core] route stop error: {error}")

    try:
        robot.stop_base()
    except Exception as error:
        print(f"[core] SDK stop error: {error}")


def _shutdown_robot(robot: GalbotRobot) -> None:
    try:
        robot.request_shutdown()
    except Exception as error:
        print(f"[core] SDK shutdown request error: {error}")

    try:
        robot.wait_for_shutdown()
    except Exception as error:
        print(f"[core] SDK shutdown wait error: {error}")

    try:
        robot.destroy()
    except Exception as error:
        print(f"[core] SDK destroy error: {error}")


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    command = PickupCommand(
        pickup_code=args.pickup_code.strip(),
        destination=args.destination.strip(),
    )

    if not args.yes and not _confirm_safety():
        print("[core] delivery flow cancelled")
        return 2

    robot = GalbotRobot()
    print("[core] initializing Galbot G1")
    if not robot.init():
        print("[core] GalbotRobot initialization failed")
        robot.destroy()
        return 1
    print("[core] GalbotRobot initialized")

    route: G1Route | None = None
    voice: G1Voice | None = None
    exit_code = 0
    try:
        voice = G1Voice(robot)
        route = G1Route(
            robot,
            timeout_s=args.timeout_s,
            step_settle_s=args.settle_s,
            controller_settle_s=args.controller_settle_s,
        )
        route.ensure_pose_controller()
        route.check_stationary()
        time.sleep(1.0)

        grabber = GrabBox(robot)
        machine = DeliveryStateMachine(
            robot,
            route,
            grabber,
            voice,
            scan_codes=args.scan_codes,
        )
        final_state = machine.start(command)
        if final_state == DeliveryState.COMPLETED:
            print("[core] delivery flow completed")
        else:
            print(
                "[core] delivery flow failed: "
                f"{machine.last_error}"
            )
            exit_code = 1
    except KeyboardInterrupt:
        print("\n[core] keyboard interrupt received")
        exit_code = 130
    except Exception as error:
        print(f"[core] delivery flow failed: {error}")
        exit_code = 1
    finally:
        _stop_voice(voice)
        _stop_motion(robot, route)
        _shutdown_robot(robot)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
