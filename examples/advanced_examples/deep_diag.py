"""Deep diagnostic: check error codes, try clear_error, and test motor response."""

import time
import numpy as np
from dexcontrol.robot import Robot
from dexcontrol.exceptions import ServiceUnavailableError

side = "left"
joint = 2

with Robot() as bot:
    arm = bot.left_arm if side == "left" else bot.right_arm

    print("\n=== BEFORE clear_error ===")

    # Error codes
    try:
        errors = arm.get_joint_err()
        for i, e in enumerate(errors):
            flag = " <<< PROBLEM JOINT" if i == joint else ""
            print(f"  joint {i}: error_code = {int(e)} (0x{int(e):08X}){flag}")
    except Exception as ex:
        print(f"  Could not get error codes: {ex}")

    # Current position
    pos = arm.get_joint_pos()
    limits = arm.joint_pos_limit
    print(f"\n  joint {joint} position: {pos[joint]:.4f} (limits: [{limits[joint][0]:.4f}, {limits[joint][1]:.4f}])")

    # Try clear_error if available
    print("\n=== Attempting clear_error ===")
    try:
        if bot.has_component("left_arm"):
            # Try the robot-level clear_error querable
            querier = bot._querables.get("clear_error")
            if querier is not None:
                print("  Found clear_error querable, calling...")
                result = querier.call({})
                print(f"  Result: {result}")
                time.sleep(2)
            else:
                print("  No clear_error querable found on robot")
    except Exception as ex:
        print(f"  clear_error failed: {ex}")

    print("\n=== AFTER clear_error ===")
    try:
        errors = arm.get_joint_err()
        for i, e in enumerate(errors):
            flag = " <<< PROBLEM JOINT" if i == joint else ""
            print(f"  joint {i}: error_code = {int(e)} (0x{int(e):08X}){flag}")
    except Exception as ex:
        print(f"  Could not get error codes: {ex}")

    # Try setting mode to position after clear
    print("\n=== Re-enabling position mode ===")
    try:
        arm.set_modes(["position"] * 7)
        print("  set_modes(position) OK")
    except Exception as ex:
        print(f"  set_modes failed: {ex}")

    time.sleep(0.5)

    # Expand limit and try to send command
    print("\n=== Sending position command (limit expanded) ===")
    original_limits = arm.joint_pos_limit.copy()
    arm._joint_pos_limit[joint] = [min(limits[joint][0], pos[joint] - 0.1),
                                    max(limits[joint][1], pos[joint] + 0.1)]

    target = limits[joint][1] - 0.05  # just inside original limit
    cmd = arm.get_joint_pos().copy()
    cmd[joint] = target
    print(f"  Commanding joint {joint} to {target:.4f}")

    arm.set_joint_pos(cmd)
    time.sleep(2)

    actual = arm.get_joint_pos()[joint]
    print(f"  After 2s: actual = {actual:.4f}")

    # Check errors again
    try:
        errors = arm.get_joint_err()
        print(f"  joint {joint} error after command: {int(errors[joint])} (0x{int(errors[joint]):08X})")
    except Exception:
        pass

    # Check current
    try:
        currents = arm.get_joint_current()
        print(f"  joint {joint} current: {currents[joint]:.4f}")
    except Exception:
        pass

    arm._joint_pos_limit[:] = original_limits
    print("\n  Limits restored.")
    print()
