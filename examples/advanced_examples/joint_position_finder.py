# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""
Interactive Joint Position Finder

Interactively select an arm and joint, type exact radian values to move
the robot, and iterate until satisfied. Useful for finding initial joint
configurations, home poses, etc.

Flow:
    1. Select arm (left / right)
    2. Select joint index (0-6)
    3. Enter radian values repeatedly — robot moves after each entry
    4. Type 'done' to pick another joint, 'back' to pick another arm

On exit the script prints a copy-paste-ready summary of all joint positions.

Usage:
    python joint_position_finder.py
    python joint_position_finder.py --wait-time 3.0
"""

import numpy as np
import tyro
from loguru import logger

from dexcontrol.robot import Robot


def _print_arm_state(arm, prefix: str, side: str, active_joint: int | None = None) -> np.ndarray:
    """Read and display all joint positions for an arm."""
    pos = np.array(arm.get_joint_pos())
    print(f"\n--- {side}_arm current state ---")
    for i in range(7):
        marker = " >>" if i == active_joint else "   "
        print(
            f"{marker}[{i}] {prefix}_arm_j{i + 1}: {pos[i]:+.4f} rad  ({np.rad2deg(pos[i]):+.2f}\u00b0)"
        )
    return pos


def main(wait_time: float = 2.0) -> None:
    """Interactive joint position finder.

    Args:
        wait_time: Seconds for the robot to reach each target position.
    """
    logger.warning("Be ready to press e-stop if needed!")
    bot = Robot()

    try:
        while True:
            side = input("\nSelect arm [left/right/done]: ").strip().lower()
            if side in ("done", "q", "quit", "exit"):
                break
            if side not in ("left", "right"):
                print("  Enter 'left', 'right', or 'done'.")
                continue

            arm = bot.left_arm if side == "left" else bot.right_arm
            prefix = "L" if side == "left" else "R"

            while True:
                _print_arm_state(arm, prefix, side)
                joint_input = input("\nSelect joint [0-6/back]: ").strip()
                if joint_input in ("back", "b"):
                    break
                if not joint_input.isdigit() or int(joint_input) > 6:
                    print("  Enter 0-6 or 'back'.")
                    continue
                joint_idx = int(joint_input)

                while True:
                    pos = _print_arm_state(arm, prefix, side, active_joint=joint_idx)
                    val = input(
                        f"\n  {prefix}_arm_j{joint_idx + 1} \u2192 enter radian value (or 'done'): "
                    ).strip()
                    if val in ("done", "d"):
                        break
                    try:
                        target = float(val)
                    except ValueError:
                        print("  Invalid number.")
                        continue

                    new_pos = pos.copy()
                    new_pos[joint_idx] = target
                    logger.info(
                        f"Moving {prefix}_arm_j{joint_idx + 1} to {target:+.4f} rad ..."
                    )
                    arm.set_joint_pos(new_pos, wait_time=wait_time)

        # --- Final summary ---
        print("\n" + "=" * 50)
        print("  Final Joint Positions")
        print("=" * 50)
        for side_name, arm_obj, pfx in [
            ("left_arm", bot.left_arm, "L"),
            ("right_arm", bot.right_arm, "R"),
        ]:
            pos = np.array(arm_obj.get_joint_pos())
            print(f"\n{side_name}:")
            print(f"  [{', '.join(f'{v:.4f}' for v in pos)}]")
            for i in range(7):
                print(f"    {pfx}_arm_j{i + 1}: {pos[i]:+.4f} rad  ({np.rad2deg(pos[i]):+.2f}\u00b0)")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        bot.shutdown()
        logger.info("Robot disconnected.")


if __name__ == "__main__":
    tyro.cli(main)
