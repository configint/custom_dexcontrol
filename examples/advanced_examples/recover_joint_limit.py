"""Recovery script for when a joint is stuck beyond its limit.

When a joint exceeds its position limit, the motor locks up and won't move
even after power cycling. This script helps recover by:

1. Showing current joint positions and which joints are over-limit
2. Releasing the brake (preferred) or disabling motors so you can
   manually move the joint back within limits
3. Re-engaging the brake / re-enabling motors after recovery

Usage:
  # Check which joints are over-limit (read-only, safe to run anytime)
  python recover_joint_limit.py status --side left

  # Release specific joints for manual recovery
  python recover_joint_limit.py release --side left --joints 2 5

  # Release ALL joints on left arm
  python recover_joint_limit.py release --side left

  # Re-engage after manually moving joints back within limits
  python recover_joint_limit.py engage --side left

WARNING: When joints are released, the arm will be limp and may fall
due to gravity. Support the arm before releasing!
"""

import logging
import sys
import time
from typing import Literal

import numpy as np
import tyro

from dexcontrol.exceptions import ServiceUnavailableError
from dexcontrol.robot import Robot

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


def _get_arm(bot: Robot, side: str):
    """Get arm component by side name."""
    if side == "left":
        return bot.left_arm
    elif side == "right":
        return bot.right_arm
    else:
        raise ValueError(f"Invalid side: {side}")


def status(
    side: Literal["left", "right"] = "right",
) -> None:
    """Show current joint positions and highlight any that are over-limit.

    This is read-only and safe to run anytime.

    Args:
        side: Which arm to check.
    """
    with Robot() as bot:
        arm = _get_arm(bot, side)
        positions = arm.get_joint_pos()
        limits = arm.joint_pos_limit  # shape (7, 2): [min, max]
        names = arm.joint_name

        print(f"\n{'='*60}")
        print(f"  {side.upper()} ARM JOINT STATUS")
        print(f"{'='*60}")
        print(f"  {'Joint':<12} {'Position':>10} {'Min':>10} {'Max':>10}  Status")
        print(f"  {'-'*56}")

        over_limit_joints = []
        for i in range(len(positions)):
            pos = positions[i]
            lo, hi = limits[i]
            name = names[i] if names else f"joint_{i}"

            if pos < lo:
                flag = " << UNDER LIMIT"
                over_limit_joints.append(i)
            elif pos > hi:
                flag = " >> OVER LIMIT"
                over_limit_joints.append(i)
            else:
                flag = " OK"

            print(f"  {name:<12} {pos:>10.4f} {lo:>10.4f} {hi:>10.4f} {flag}")

        print(f"{'='*60}")
        if over_limit_joints:
            print(f"\n  Joints over limit: {over_limit_joints}")
            print(f"  To recover, run:")
            print(f"    python {__file__} release --side {side} --joints {' '.join(str(j) for j in over_limit_joints)}")
        else:
            print(f"\n  All joints are within limits.")
        print()


def release(
    side: Literal["left", "right"] = "left",
    joints: list[int] | None = None,
) -> None:
    """Release joints so they can be manually moved back within limits.

    Tries release_brake first (full free-drive). If that service is
    unavailable, falls back to set_modes(disable) which removes motor
    torque but may keep the mechanical brake engaged.

    Args:
        side: Which arm to release.
        joints: Specific joint indices (0-6) to release. If None, releases all.
    """
    if joints is None:
        joints = list(range(7))

    for j in joints:
        if j < 0 or j > 6:
            logger.error(f"Invalid joint index: {j}. Must be 0-6.")
            sys.exit(1)

    with Robot() as bot:
        arm = _get_arm(bot, side)

        # Show current status first
        positions = arm.get_joint_pos()
        limits = arm.joint_pos_limit
        names = arm.joint_name

        print(f"\n  Joints to release on {side} arm: {joints}")
        for j in joints:
            pos = positions[j]
            lo, hi = limits[j]
            name = names[j] if names else f"joint_{j}"
            in_limit = "OK" if lo <= pos <= hi else "OVER LIMIT"
            print(f"    [{j}] {name}: {pos:.4f}  (limits: [{lo:.4f}, {hi:.4f}])  {in_limit}")

        print(f"\n  WARNING: The arm may fall due to gravity when released!")
        print(f"  Make sure you are supporting the arm before continuing.")
        confirm = input("\n  Continue? [y/N]: ").strip().lower()
        if confirm != "y":
            print("  Aborted.")
            return

        # Step 1: Disable motor torque (so motor stops holding position)
        modes = ["position"] * 7
        for j in joints:
            modes[j] = "disable"
        try:
            arm.set_modes(modes)
            logger.info(f"Motor torque disabled for joints {joints}.")
        except Exception as e:
            logger.error(f"Failed to disable motors: {e}")
            sys.exit(1)

        # Step 2: Release mechanical brake (so joint can move freely)
        brake_released = False
        try:
            logger.info(f"Attempting brake release for {side} arm joints {joints}...")
            result = arm.release_brake(enable=True, joints=joints)
            if result.get("success", False):
                brake_released = True
                logger.info(f"Brake released successfully: {result.get('message', '')}")
            else:
                logger.warning(f"Brake release returned failure: {result.get('message', '')}")
        except ServiceUnavailableError:
            logger.warning(f"Brake service not available for {side} arm. Motor torque is off but mechanical brake may resist.")
        except Exception as e:
            logger.warning(f"Brake release failed: {e}. Motor torque is off but mechanical brake may resist.")

        print(f"\n  Joints released. Manually move the arm back within limits.")
        print(f"  When done, run:")
        print(f"    python {__file__} engage --side {side}")
        print()

        # Keep the script running so the robot connection stays alive
        print("  Press Ctrl+C when you're done moving the arm...")
        try:
            while True:
                time.sleep(1)
                # Periodically show position
                positions = arm.get_joint_pos()
                status_parts = []
                for j in joints:
                    pos = positions[j]
                    lo, hi = limits[j]
                    if lo <= pos <= hi:
                        status_parts.append(f"  j{j}: {pos:.3f} OK")
                    else:
                        status_parts.append(f"  j{j}: {pos:.3f} !!")
                print(f"\r  {' | '.join(status_parts)}", end="", flush=True)
        except KeyboardInterrupt:
            print("\n")
            logger.info("Stopping...")

            # Re-engage brake first, then re-enable motors
            if brake_released:
                try:
                    arm.release_brake(enable=False)
                    logger.info("Brake re-engaged.")
                except Exception:
                    logger.warning("Could not re-engage brake automatically.")

            try:
                arm.set_modes(["position"] * 7)
                logger.info("All motors re-enabled in position mode.")
            except Exception:
                logger.warning("Could not re-enable motors automatically.")

            # Final status check
            positions = arm.get_joint_pos()
            all_ok = True
            for j in joints:
                pos = positions[j]
                lo, hi = limits[j]
                if pos < lo or pos > hi:
                    all_ok = False
                    name = names[j] if names else f"joint_{j}"
                    logger.warning(f"  {name} (j{j}) is still over limit: {pos:.4f} (limits: [{lo:.4f}, {hi:.4f}])")

            if all_ok:
                logger.info("All released joints are now within limits!")
            else:
                logger.warning("Some joints are still over limit. You may need to run this again.")


def engage(
    side: Literal["left", "right"] = "left",
) -> None:
    """Re-engage motors and brakes after manual recovery.

    Args:
        side: Which arm to re-engage.
    """
    with Robot() as bot:
        arm = _get_arm(bot, side)

        # Try to disable brake release
        try:
            result = arm.release_brake(enable=False)
            logger.info(f"Brake re-engaged: {result.get('message', '')}")
        except (ServiceUnavailableError, Exception) as e:
            logger.info(f"Brake service not used (may not be needed): {e}")

        # Re-enable all motors in position mode
        try:
            arm.set_modes(["position"] * 7)
            logger.info(f"All {side} arm motors re-enabled in position mode.")
        except Exception as e:
            logger.error(f"Failed to re-enable motors: {e}")
            sys.exit(1)

        # Show final status
        positions = arm.get_joint_pos()
        limits = arm.joint_pos_limit
        names = arm.joint_name
        print(f"\n  Final joint positions ({side} arm):")
        all_ok = True
        for i in range(len(positions)):
            pos = positions[i]
            lo, hi = limits[i]
            name = names[i] if names else f"joint_{i}"
            if pos < lo or pos > hi:
                all_ok = False
                print(f"    [{i}] {name}: {pos:.4f}  STILL OVER LIMIT [{lo:.4f}, {hi:.4f}]")
            else:
                print(f"    [{i}] {name}: {pos:.4f}  OK")

        if all_ok:
            print(f"\n  All joints within limits. Robot should be operational.")
        else:
            print(f"\n  Some joints still over limit. Robot may not move properly.")
        print()


if __name__ == "__main__":
    tyro.extras.subcommand_cli_from_dict(
        {"status": status, "release": release, "engage": engage}
    )
