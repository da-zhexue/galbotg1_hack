
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from galbot_sdk.g1 import GalbotRobot, ControlStatus


# ==========================================
# Galbot G1 Sequential Movement
# ==========================================

robot = GalbotRobot()
initialized = False

# Each step: name, x, y, yaw, interpolation_time
# Right movement uses negative Y in robot coordinates.

ROUTE = [
    ("Forward 80cm",  0.80,  0.00, 0.0, 6.0),
    ("Right 50cm - 1", 0.00, -0.50, 0.0, 5.0),
    ("Right 50cm - 2", 0.00, -0.50, 0.0, 5.0),
    ("Right 50cm - 3", 0.00, -0.50, 0.0, 5.0),
    ("Right 50cm - 4", 0.00, -0.50, 0.0, 5.0),
    ("Forward 100cm", 1.00,  0.00, 0.0, 8.0),
]

TIMEOUT = 25.0


def check_stationary():
    velocity = robot.get_base_velocity()

    linear = velocity["linear_velocity"]
    angular = velocity["angular_velocity"]

    if max(abs(float(v)) for v in linear) > 0.03:
        raise RuntimeError("Base is not stationary")

    if max(abs(float(v)) for v in angular) > 0.05:
        raise RuntimeError("Base is rotating")


def execute_step(index, step):
    name, x, y, yaw, duration = step

    print("\n================================")
    print("Step {}/{}".format(index, len(ROUTE)))
    print("Action:", name)
    print("X:", x, "m")
    print("Y:", y, "m")
    print("Yaw:", yaw, "rad")
    print("Duration:", duration, "s")
    print("================================")

    status = robot.set_base_pose(
        x,
        y,
        yaw,
        "rel(0)",
        "odom",
        duration,
        True,
        TIMEOUT
    )

    print("Motion status:", status)

    if status != ControlStatus.SUCCESS:
        raise RuntimeError(
            "Step {} failed: {}".format(index, status)
        )

    print("Step completed:", name)

    time.sleep(0.5)

    print("Current odometry:")
    print(robot.get_odom())


def main():
    global initialized

    try:
        print("Initializing Galbot G1...")

        initialized = bool(robot.init())

        if not initialized:
            raise RuntimeError("G1 initialization failed")

        print("G1 initialized successfully!")

        time.sleep(1)

        # Check controller
        controller = robot.get_active_controller("chassis")
        print("Current controller:", controller)

        if controller != "chassis_pose_ctrl":

            print("Switching to chassis_pose_ctrl...")

            status = robot.switch_controller(
                "chassis_pose_ctrl"
            )

            print("Switch status:", status)

            if status != ControlStatus.SUCCESS:
                raise RuntimeError("Controller switch failed")

            time.sleep(1)

            controller = robot.get_active_controller("chassis")

            if controller != "chassis_pose_ctrl":
                raise RuntimeError("Pose controller not confirmed")

        print("Pose controller ready!")

        # Check initial state
        check_stationary()

        print("\nInitial odometry:")
        print(robot.get_odom())

        print("\n========== ROUTE ==========")

        for i, step in enumerate(ROUTE, 1):
            print("{}. {}".format(i, step[0]))

        print("\nTotal path length: 3.8 m")
        print("===========================")

        confirm = input(
            "\nConfirm 3.8m route is clear. Type GO: "
        ).strip()

        if confirm != "GO":
            print("Route cancelled")
            return

        # Execute route sequentially
        for i, step in enumerate(ROUTE, 1):

            check_stationary()

            execute_step(i, step)

        print("\n================================")
        print("ALL SIX MOVEMENTS COMPLETED!")
        print("G1 route execution finished.")
        print("================================")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")

    except Exception as e:
        print("\nERROR:", repr(e))
        print("Remaining route cancelled")

    finally:
        if initialized:

            try:
                print("\nStopping base...")
                status = robot.stop_base()
                print("Stop status:", status)

                if status != ControlStatus.SUCCESS:
                    print("WARNING: Check physical emergency stop!")

            except Exception as e:
                print("Stop error:", repr(e))

            try:
                robot.request_shutdown()
                robot.wait_for_shutdown()
                robot.destroy()

            except Exception as e:
                print("SDK shutdown error:", repr(e))

        print("Program finished")


if __name__ == "__main__":
    main()
