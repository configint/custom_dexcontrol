"""Diagnose a single joint - check errors, mode, position, current, and try a small move."""

import time
from typing import Literal

import numpy as np
import tyro
from loguru import logger

from dexcontrol.robot import Robot


def main(
    side: Literal["left", "right"] = "left",
    joint: int = 2,
    nudge: float = 0.1,
) -> None:
    """Diagnose a joint and attempt a small move.

    Args:
        side: Which arm.
        joint: Joint index (0-6).
        nudge: How far to try moving (rad).
    """
    with Robot() as bot:
        arm = bot.left_arm if side == "left" else bot.right_arm
        part = f"{side}_arm"
        names = arm.joint_name

        j = joint
        name = names[j] if names else f"joint_{j}"
        print(f"\n{'='*50}")
        print(f"  Diagnosing {side} arm joint {j} ({name})")
        print(f"{'='*50}")

        # 1. Position & limits
        pos = arm.get_joint_pos()
        limits = arm.joint_pos_limit
        lo, hi = limits[j]
        print(f"\n  Position:  {pos[j]:.4f}")
        print(f"  Limits:    [{lo:.4f}, {hi:.4f}]")
        in_limit = "YES" if lo <= pos[j] <= hi else "NO"
        print(f"  In limit:  {in_limit}")

        # 2. Current
        try:
            cur = arm.get_joint_current()
            print(f"  Current:   {cur[j]:.4f}")
        except Exception as e:
            print(f"  Current:   ERROR ({e})")

        # 3. Error state
        try:
            state = arm._get_state()
            errors = state.get("error", {})
            print(f"\n  Errors (all joints): {errors}")
            if j in errors:
                print(f"  >>> Joint {j} error: {errors[j]}")
            elif str(j) in errors:
                print(f"  >>> Joint {j} error: {errors[str(j)]}")
            else:
                print(f"  >>> Joint {j}: no error")
        except Exception as e:
            print(f"  Error state: could not read ({e})")

        # 4. Try clear_error
        print(f"\n  Clearing errors on {part}...")
        try:
            bot.clear_error(part)
            print(f"  clear_error: OK")
        except Exception as e:
            print(f"  clear_error: {e}")

        # Re-check error
        try:
            state = arm._get_state()
            errors = state.get("error", {})
            print(f"  Errors after clear: {errors}")
        except Exception:
            pass

        # 5. Set position mode
        print(f"\n  Setting all joints to position mode...")
        try:
            arm.set_modes(["position"] * 7)
            print(f"  set_modes: OK")
        except Exception as e:
            print(f"  set_modes: {e}")

        # 6. Try small move
        target = pos.copy()
        target[j] = pos[j] + nudge
        # Clamp to limits
        target[j] = np.clip(target[j], lo + 0.02, hi - 0.02)

        print(f"\n  Attempting move: {pos[j]:.4f} -> {target[j]:.4f} (delta={target[j]-pos[j]:.4f})")
        confirm = input("  Continue? [y/N]: ").strip().lower()
        if confirm != "y":
            print("  Skipped.")
            return

        try:
            arm.set_joint_pos(target, wait_time=3.0)
        except Exception as e:
            print(f"  set_joint_pos error: {e}")

        time.sleep(0.5)
        new_pos = arm.get_joint_pos()
        try:
            new_cur = arm.get_joint_current()[j]
        except Exception:
            new_cur = float("nan")

        moved = new_pos[j] - pos[j]
        print(f"\n  Result:")
        print(f"    Before:  {pos[j]:.4f}")
        print(f"    After:   {new_pos[j]:.4f}")
        print(f"    Moved:   {moved:.4f} rad ({np.degrees(moved):.2f} deg)")
        print(f"    Current: {new_cur:.4f}")

        if abs(moved) < 0.001:
            print(f"\n  Joint did NOT move. Possible causes:")
            print(f"    - Firmware error still active")
            print(f"    - Motor hardware issue")
            print(f"    - Joint mechanically stuck")

            # Check error again
            try:
                state = arm._get_state()
                errors = state.get("error", {})
                if errors:
                    print(f"    - Current errors: {errors}")
            except Exception:
                pass
        else:
            print(f"\n  Joint moved successfully!")
        print()


if __name__ == "__main__":
    tyro.cli(main)
