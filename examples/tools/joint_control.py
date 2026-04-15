"""Interactive joint control tool for debugging and waypoint tuning.

Read current joint positions and move individual or all joints interactively.

Usage:
    python joint_control.py --arm-side left
    python joint_control.py --arm-side right
"""

from typing import Literal

import numpy as np
import tyro

from dexcontrol.robot import Robot


def print_joints(arm, arm_side: str) -> np.ndarray:
    pos = arm.get_joint_pos()
    print(f"\n[{arm_side.capitalize()} Arm Joint Positions]")
    parts = [f"  j{i}: {pos[i]:+.4f}" for i in range(len(pos))]
    print("  ".join(parts))
    return pos


def print_help():
    print(
        "\nCommands:\n"
        "  r              — refresh (현재 값 읽기)\n"
        "  s <idx> <val>  — 단일 관절 이동 (예: s 4 0.5)\n"
        "  a <v0> ... <v6> — 전체 관절 이동\n"
        "  c              — 현재 값을 np.array 포맷으로 출력\n"
        "  h              — 도움말\n"
        "  q              — 종료"
    )


def main(
    arm_side: Literal["left", "right"] = "left",
    wait_time: float = 3.0,
) -> None:
    """Interactive joint control.

    Args:
        arm_side: Which arm to control.
        wait_time: Seconds for interpolated motion (safety).
    """
    bot = Robot()
    arm = bot.left_arm if arm_side == "left" else bot.right_arm

    print_joints(arm, arm_side)
    print_help()

    try:
        while True:
            try:
                cmd = input("\n> ").strip()
            except EOFError:
                break
            if not cmd:
                continue

            parts = cmd.split()
            op = parts[0].lower()

            if op == "q":
                break
            elif op == "r":
                print_joints(arm, arm_side)
            elif op == "h":
                print_help()
            elif op == "c":
                pos = arm.get_joint_pos()
                formatted = ", ".join(f"{v:.4f}" for v in pos)
                print(f"\nnp.array([{formatted}])")
            elif op == "s":
                if len(parts) != 3:
                    print("Usage: s <joint_idx> <value>")
                    continue
                try:
                    idx = int(parts[1])
                    val = float(parts[2])
                except ValueError:
                    print("Invalid idx or value")
                    continue
                if not 0 <= idx <= 6:
                    print("Joint index must be 0-6")
                    continue
                pos = arm.get_joint_pos().copy()
                pos[idx] = val
                print(f"Moving j{idx} to {val:.4f} (wait={wait_time}s) ...")
                arm.set_joint_pos(pos, wait_time=wait_time)
                print_joints(arm, arm_side)
            elif op == "a":
                if len(parts) != 8:
                    print("Usage: a <v0> <v1> <v2> <v3> <v4> <v5> <v6>")
                    continue
                try:
                    vals = np.array([float(v) for v in parts[1:]])
                except ValueError:
                    print("Invalid values")
                    continue
                print(f"Moving all joints (wait={wait_time}s) ...")
                arm.set_joint_pos(vals, wait_time=wait_time)
                print_joints(arm, arm_side)
            else:
                print(f"Unknown command: {op}  (h for help)")
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        bot.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
