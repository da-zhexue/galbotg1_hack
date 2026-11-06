"""Interactive real-robot test for the ROS-free grabbox package."""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence
from pathlib import Path

from galbot_sdk.g1 import GalbotRobot

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from grabbox.controller import GrabboxError, GrabBox
else:
    from .controller import GrabboxError, GrabBox


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run grabbox state tests on a real Galbot G1 robot."
        )
    )
    parser.add_argument(
        "--state",
        choices=("all", "default", "grab", "drop"),
        default="all",
        help="State to test. 'all' runs default, grab, drop.",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=2.0,
        help="Pause between states in seconds.",
    )
    parser.add_argument(
        "--restore",
        action="store_true",
        help="Return to default after the selected state.",
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
    print("  3. Clear people and obstacles from the robot workspace.")
    print("  4. Keep the emergency stop within reach.")
    answer = input("Type YES to start the real-robot test: ").strip()
    return answer == "YES"


def _run_state(
    box: GrabBox,
    state: str,
    settle_s: float,
    sleep: Callable[[float], None],
) -> None:
    print(f"[grabbox] executing state: {state}")
    if state == "default":
        box.default()
    elif state == "grab":
        box.grab()
    elif state == "drop":
        box.drop()
    else:
        raise ValueError(f"unsupported state: {state}")
    sleep(max(settle_s, 0.0))
    print(f"[grabbox] state completed: {state}")


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)

    if not args.yes and not _confirm_safety():
        print("[grabbox] test cancelled")
        return 2

    robot = GalbotRobot()
    if not robot.init():
        print("[grabbox] GalbotRobot initialization failed")
        robot.destroy()
        return 1

    states = (
        ["default", "grab", "drop"]
        if args.state == "all"
        else [args.state]
    )
    if args.restore and states[-1] != "default":
        states.append("default")

    try:
        box = GrabBox(robot)
        print("[grabbox] waiting for robot data")
        time.sleep(2.0)
        for state in states:
            _run_state(
                box,
                state,
                args.settle_s,
                time.sleep,
            )
        return 0
    except (GrabboxError, RuntimeError) as error:
        print(f"[grabbox] test failed: {error}")
        robot.stop_trajectory_execution()
        robot.stop_base()
        return 1
    finally:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()


if __name__ == "__main__":
    raise SystemExit(main())