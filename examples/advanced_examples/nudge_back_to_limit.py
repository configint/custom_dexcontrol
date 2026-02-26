"""Nudge an over-limit joint back within limits using position commands.

When a joint is slightly over limit and the brake service is unavailable,
this script temporarily expands the software limit to include the current
position, then commands the joint to move back within the original limit.

Usage:
  python nudge_back_to_limit.py --side left --joint 2
"""

import sys
import time
from typing import Literal

import numpy as np
import tyro
from loguru import logger

from dexcontrol.robot import Robot


def main(
    side: Literal["left", "right"] = "left",
    joint: int = 2,
    margin: float = 0.05,
    speed: float = 0.1,
) -> None:
    """Nudge an over-limit joint back within its limits.

    Args:
        side: Which arm.
        joint: Joint index (0-6) that is over limit.
        margin: How far inside the limit to target (rad).
        speed: How fast to move (rad/s equivalent, lower = gentler).
    """
    if joint < 0 or joint > 6:
        logger.error(f"Invalid joint index: {joint}. Must be 0-6.")
        sys.exit(1)

    logger.warning("WARNING: This will command the motor to move.")
    logger.warning("Be ready to press e-stop if needed!")
    if input("Continue? [y/N]: ").strip().lower() != "y":
        return

    with Robot() as bot:
        arm = bot.left_arm if side == "left" else bot.right_arm

        positions = arm.get_joint_pos()
        limits = arm.joint_pos_limit  # shape (7, 2)
        names = arm.joint_name

        pos = positions[joint]
        lo, hi = limits[joint]
        name = names[joint] if names else f"joint_{joint}"

        logger.info(f"{name}: position={pos:.4f}, limits=[{lo:.4f}, {hi:.4f}]")

        if lo <= pos <= hi:
            logger.info(f"{name} is already within limits. Nothing to do.")
            return

        # Determine target: move to just inside the original limit
        if pos > hi:
            target = hi - margin
            logger.info(f"{name} is {pos - hi:.4f} rad over upper limit.")
        else:
            target = lo + margin
            logger.info(f"{name} is {lo - pos:.4f} rad under lower limit.")

        logger.info(f"Target position: {target:.4f}")

        # Temporarily patch the software limit to include current position
        # so set_joint_pos won't clip it to a no-op
        original_limits = arm.joint_pos_limit.copy()
        expanded_min = min(lo, pos - 0.01)
        expanded_max = max(hi, pos + 0.01)
        arm._joint_pos_limit[joint] = [expanded_min, expanded_max]
        logger.info(f"Temporarily expanded limits for {name}: [{expanded_min:.4f}, {expanded_max:.4f}]")

        try:
            # First make sure motor is in position mode
            modes = ["position"] * 7
            arm.set_modes(modes)
            logger.info("Motors set to position mode.")

            # Generate a slow trajectory from current pos to target
            steps = max(int(abs(pos - target) / speed * 50), 20)
            logger.info(f"Moving {name} from {pos:.4f} to {target:.4f} in {steps} steps...")

            current_positions = arm.get_joint_pos().copy()
            for i in range(steps + 1):
                alpha = i / steps
                interp = pos + (target - pos) * alpha
                cmd = current_positions.copy()
                cmd[joint] = interp
                arm.set_joint_pos(cmd)
                time.sleep(0.02)

                # Read actual position
                if i % 10 == 0:
                    actual = arm.get_joint_pos()[joint]
                    logger.info(f"  step {i}/{steps}: cmd={interp:.4f}, actual={actual:.4f}")

            time.sleep(0.5)
            final_pos = arm.get_joint_pos()[joint]
            logger.info(f"Final position: {final_pos:.4f} (limit: [{lo:.4f}, {hi:.4f}])")

            if lo <= final_pos <= hi:
                logger.info(f"{name} is now within limits!")
            else:
                logger.warning(f"{name} is still over limit. The motor may not be responding.")
                logger.warning("You may need to update firmware to enable brake release.")

        finally:
            # Restore original limits
            arm._joint_pos_limit[:] = original_limits
            logger.info("Software limits restored.")


if __name__ == "__main__":
    tyro.cli(main)
