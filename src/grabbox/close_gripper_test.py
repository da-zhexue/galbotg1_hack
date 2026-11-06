#!/usr/bin/env python3
"""Close a G1 gripper through the Galbot SDK.

The script is safe by default and does not command the robot unless
``--execute`` is supplied. Run it from an environment that has sourced the
Galbot SDK setup script, for example:

    source /path/to/galbot_sdk/linux-x86_64-gcc940/setup.sh
    python3 src/grabbox/close_gripper_test.py --execute --side left
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Sequence

import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotRobot, G1ControllerName


SIDES = ("left", "right", "both")
MAX_G1_GRIPPER_WIDTH_M = 0.12
MAX_GRIPPER_VELOCITY_MPS = 0.2
MAX_GRIPPER_EFFORT = 100.0
GRIPPER_CONTROLLERS = {
    "left_gripper": G1ControllerName.LEFT_GRIPPER_CTRL,
    "right_gripper": G1ControllerName.RIGHT_GRIPPER_CTRL,
}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Close the left, right, or both G1 grippers."
    )
    parser.add_argument(
        "--side",
        choices=SIDES,
        default="left",
        help="Gripper side to close (default: left).",
    )
    parser.add_argument(
        "--width",
        type=float,
        default=0.0,
        help="Target width in meters (default: 0.0, fully closed).",
    )
    parser.add_argument(
        "--velocity",
        type=float,
        default=0.03,
        help="Motion speed in m/s (default: 0.03).",
    )
    parser.add_argument(
        "--effort",
        type=float,
        default=5.0,
        help="Maximum gripper effort in N*m (default: 5.0).",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually command the gripper; without this flag only validate.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the interactive safety confirmation.",
    )
    parser.add_argument(
        "--no-switch-controller",
        action="store_true",
        help="Do not switch to the matching gripper controller before execution.",
    )
    args = parser.parse_args(argv)

    if not math.isfinite(args.width) or not 0.0 <= args.width <= MAX_G1_GRIPPER_WIDTH_M:
        parser.error(
            f"--width must be finite and in [0.0, {MAX_G1_GRIPPER_WIDTH_M}]"
        )
    if (
        not math.isfinite(args.velocity)
        or not 0.0 < args.velocity <= MAX_GRIPPER_VELOCITY_MPS
    ):
        parser.error(
            f"--velocity must be finite and in (0.0, {MAX_GRIPPER_VELOCITY_MPS}]"
        )
    if not math.isfinite(args.effort) or not 0.0 < args.effort <= MAX_GRIPPER_EFFORT:
        parser.error(f"--effort must be finite and in (0.0, {MAX_GRIPPER_EFFORT}]")
    return args


def selected_effectors(side: str) -> list[str]:
    if side == "both":
        return ["left_gripper", "right_gripper"]
    return [f"{side}_gripper"]


def confirm_execution(args: argparse.Namespace, effectors: Sequence[str]) -> bool:
    if not args.execute or args.yes:
        return True

    print("About to command:", ", ".join(effectors))
    print(
        f"target_width={args.width:.3f} m, "
        f"velocity={args.velocity:.3f} m/s, effort={args.effort:.1f} N*m"
    )
    print(
        "Confirm the emergency stop is released, the gripper path is clear, "
        "and the robot is in a supported operating mode."
    )
    return input("Type CLOSE to continue: ").strip() == "CLOSE"


def read_state(robot: GalbotRobot, effector: str):
    state = robot.get_gripper_state(effector)
    if state is None:
        raise RuntimeError(f"No gripper feedback available for {effector}")
    return state


def describe_state(state) -> str:
    return (
        f"width={state.width:.4f} m, "
        f"velocity={state.velocity:.4f} m/s, "
        f"effort={state.effort:.2f} N*m, "
        f"is_moving={state.is_moving}"
    )


def close_grippers(args: argparse.Namespace, effectors: Sequence[str]) -> int:
    robot = GalbotRobot()
    initialized = False
    try:
        if not robot.init():
            robot.destroy()
            raise RuntimeError("GalbotRobot initialization failed")
        initialized = True

        # Check every requested gripper before sending the first command so a
        # two-sided request never moves only one side due to missing feedback.
        states = {}
        for effector in effectors:
            state = read_state(robot, effector)
            states[effector] = state
            print(f"{effector} initial: {describe_state(state)}")

        for effector in effectors:
            if not args.no_switch_controller:
                controller = GRIPPER_CONTROLLERS[effector]
                controller_status = robot.switch_controller(controller)
                print(f"{effector} controller {controller}: {controller_status}")
                if controller_status != gm.ControlStatus.SUCCESS:
                    return 1

            status = robot.set_gripper_command(
                end_effector=effector,
                width_m=args.width,
                velocity_mps=args.velocity,
                effort=args.effort,
                is_blocking=True,
            )
            print(f"{effector} command status: {status}")
            if status != gm.ControlStatus.SUCCESS:
                return 1

        time.sleep(0.5)
        for effector in effectors:
            state = read_state(robot, effector)
            print(f"{effector} final: {describe_state(state)}")

        return 0
    finally:
        if initialized:
            robot.destroy()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    effectors = selected_effectors(args.side)

    print(f"Grippers: {', '.join(effectors)}")
    print(f"Target width: {args.width:.3f} m")
    if not args.execute:
        print("Validation only. Add --execute to command the gripper.")
        return 0

    if not confirm_execution(args, effectors):
        print("Cancelled.")
        return 2

    try:
        return close_grippers(args, effectors)
    except Exception as exc:
        print(f"Close gripper test failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
