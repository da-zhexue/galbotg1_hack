#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive real-robot test for the Galbot G1 chassis route."""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence

from galbot_sdk.g1 import GalbotNavigation, GalbotRobot

if __package__:
    from .g1_route import G1Route, G1RouteError, check_numpy_available
else:
    from g1_route import G1Route, G1RouteError, check_numpy_available


REAL_TEST_ACTIONS = (
    "all",
    "forward-60cm",
    "right-50cm",
    "right-55cm",
    "forward-40cm",
    "leave-table",
    "forward-200cm",
    "fixed-point-1",
    "fixed-point-2",
    "fixed-point-3",
    "return-origin",
    "navigate-door-100cm",
)

ALL_ACTION_IDS = (
    "forward-60cm",
    "right-50cm",
    "right-55cm",
    "forward-40cm",
    "leave-table",
    "forward-200cm",
    "fixed-point-1",
    "fixed-point-2",
    "fixed-point-3",
    "return-origin",
)

NEEDS_NAVIGATION = frozenset({"navigate-door-100cm"})


def _parse_args(
    argv: Sequence[str] | None = None,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run G1 chassis route actions on a real robot."
    )
    parser.add_argument(
        "--action",
        choices=REAL_TEST_ACTIONS,
        default="all",
        help="Action to test. 'all' runs the complete route.",
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
        help="Pause after each action in seconds.",
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
    answer = input("Type YES to start the real-robot test: ").strip()
    return answer == "YES"


def _actions(
    route: G1Route,
) -> dict[str, tuple[str, Callable[[], None]]]:
    return {
        "forward-60cm": ("Forward 60cm", route.forward_60cm),
        "right-50cm": ("Right 50cm", route.right_50cm),
        "right-55cm": ("Right 55cm", route.right_55cm),
        "forward-40cm": ("Forward 40cm", route.forward_40cm),
        "leave-table": (
            "Leave table 100cm",
            lambda: route.leave_table(1.00),
        ),
        "forward-200cm": ("Forward 200cm", route.forward_200cm),
        "fixed-point-1": ("Fixed point 1", route.fixed_point_1),
        "fixed-point-2": ("Fixed point 2", route.fixed_point_2),
        "fixed-point-3": ("Fixed point 3", route.fixed_point_3),
        "return-origin": ("Return to origin", route.return_to_origin),
        "navigate-door-100cm": (
            "Navigate Door 100cm",
            route.navigate_to_door,
        ),
    }


def _run_test_actions(route: G1Route, selection: str) -> None:
    actions = _actions(route)
    if selection == "all":
        selected = [
            (action_id, actions[action_id])
            for action_id in ALL_ACTION_IDS
        ]
    else:
        selected = [(selection, actions[selection])]

    for index, (action_id, (name, action)) in enumerate(
        selected,
        start=1,
    ):
        route.check_stationary()
        print(
            f"\n[g1-route] action {index}/{len(selected)}: "
            f"{action_id} ({name})"
        )
        action()
        print(f"[g1-route] action completed: {action_id}")


def _stop_navigation(navigation: GalbotNavigation | None) -> None:
    if navigation is None:
        return
    try:
        success, status = navigation.stop_navigation()
        print(f"[g1-route] navigation stop: {success} {status}")
    except Exception as error:
        print(f"[g1-route] navigation stop error: {error}")


def _stop_robot(robot: GalbotRobot, route: G1Route | None) -> None:
    if route is not None:
        try:
            route.stop_base()
            return
        except Exception as error:
            print(f"[g1-route] route stop error: {error}")

    try:
        robot.stop_base()
    except Exception as error:
        print(f"[g1-route] SDK stop error: {error}")


def _shutdown_robot(robot: GalbotRobot) -> None:
    try:
        robot.request_shutdown()
    except Exception as error:
        print(f"[g1-route] SDK shutdown request error: {error}")

    try:
        robot.wait_for_shutdown()
    except Exception as error:
        print(f"[g1-route] SDK shutdown wait error: {error}")

    try:
        robot.destroy()
    except Exception as error:
        print(f"[g1-route] SDK destroy error: {error}")


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    if not args.yes and not _confirm_safety():
        print("[g1-route] test cancelled")
        return 2

    if args.action in NEEDS_NAVIGATION:
        try:
            check_numpy_available()
        except G1RouteError as error:
            print(f"[g1-route] {error}")
            print(
                "[g1-route] install a supported numpy for "
                f"{sys.executable}, for example:"
            )
            print(
                f"  {sys.executable} -m pip install --user "
                "'numpy==1.24.4'"
            )
            return 1

    robot = GalbotRobot()
    print("[g1-route] initializing Galbot G1")
    if not robot.init():
        print("[g1-route] GalbotRobot initialization failed")
        robot.destroy()
        return 1
    print("[g1-route] GalbotRobot initialized")

    navigation: GalbotNavigation | None = None
    if args.action in NEEDS_NAVIGATION:
        try:
            navigation = GalbotNavigation()
            print("[g1-route] initializing GalbotNavigation")
            if not navigation.init():
                print("[g1-route] GalbotNavigation initialization failed")
                navigation = None
        except Exception as error:
            print(f"[g1-route] GalbotNavigation init error: {error}")
            navigation = None
        if navigation is None:
            _shutdown_robot(robot)
            return 1

    route: G1Route | None = None
    exit_code = 0
    try:
        route = G1Route(
            robot,
            timeout_s=args.timeout_s,
            step_settle_s=args.settle_s,
            controller_settle_s=args.controller_settle_s,
            navigation=navigation,
        )
        time.sleep(1.0)
        route.ensure_pose_controller()
        route.check_stationary()

        print("[g1-route] initial odometry:")
        print(robot.get_odom())

        _run_test_actions(route, args.action)
        print("\n[g1-route] real-robot test completed")
    except KeyboardInterrupt:
        print("\n[g1-route] keyboard interrupt received")
        exit_code = 130
    except Exception as error:
        print(f"[g1-route] test failed: {error}")
        exit_code = 1
    finally:
        _stop_navigation(navigation)
        _stop_robot(robot, route)
        _shutdown_robot(robot)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
