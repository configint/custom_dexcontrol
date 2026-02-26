"""Quick diagnostic: dump arm joint state, brake status, and error codes."""

import sys
from dexcontrol.robot import Robot
from dexcontrol.exceptions import ServiceUnavailableError

side = sys.argv[1] if len(sys.argv) > 1 else "left"

with Robot() as bot:
    arm = bot.left_arm if side == "left" else bot.right_arm

    names = arm.joint_name
    positions = arm.get_joint_pos()
    limits = arm.joint_pos_limit
    velocities = arm.get_joint_vel()

    try:
        torques = arm.get_joint_torque()
    except Exception:
        torques = None

    try:
        currents = arm.get_joint_current()
    except Exception:
        currents = None

    try:
        errors = arm.get_joint_err()
    except Exception:
        errors = None

    # Brake status
    brake_status = None
    try:
        brake_status = arm.get_brake_status()
    except ServiceUnavailableError as e:
        brake_status = f"ServiceUnavailable: {e}"
    except Exception as e:
        brake_status = f"Error: {e}"

    print(f"\n{'='*70}")
    print(f"  {side.upper()} ARM DIAGNOSTIC")
    print(f"{'='*70}")

    print(f"\n  Brake status: {brake_status}")

    print(f"\n  {'Joint':<12} {'Pos':>8} {'Min':>8} {'Max':>8} {'Vel':>8}", end="")
    if torques is not None:
        print(f" {'Torque':>8}", end="")
    if currents is not None:
        print(f" {'Current':>8}", end="")
    if errors is not None:
        print(f" {'Error':>8}", end="")
    print(f"  {'Status'}")
    print(f"  {'-'*66}")

    for i in range(len(positions)):
        name = names[i] if names else f"joint_{i}"
        pos = positions[i]
        lo, hi = limits[i]
        vel = velocities[i]

        line = f"  {name:<12} {pos:>8.4f} {lo:>8.4f} {hi:>8.4f} {vel:>8.4f}"
        if torques is not None:
            line += f" {torques[i]:>8.4f}"
        if currents is not None:
            line += f" {currents[i]:>8.4f}"
        if errors is not None:
            line += f" {int(errors[i]):>8d}"

        if pos < lo or pos > hi:
            line += "  !! OVER LIMIT"
        else:
            line += "  OK"

        print(line)

    print(f"{'='*70}\n")
