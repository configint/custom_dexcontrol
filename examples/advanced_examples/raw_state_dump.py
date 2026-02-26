"""Dump raw joint state and try to find clear_error service."""

import time
from dexcontrol.robot import Robot

side = "left"

with Robot() as bot:
    arm = bot.left_arm if side == "left" else bot.right_arm

    # Dump raw state
    print("\n=== Raw joint state ===")
    state = arm._get_state()
    for key, val in state.items():
        print(f"  {key}: {val}")

    # Check what querables the robot has
    print("\n=== Robot querables ===")
    try:
        # Look at the robot's internal structure for querables
        for attr in dir(bot):
            if "quer" in attr.lower() or "clear" in attr.lower() or "error" in attr.lower():
                print(f"  bot.{attr} = {getattr(bot, attr, 'N/A')}")
    except Exception as ex:
        print(f"  Error: {ex}")

    # Try to find the clear_error service via the robot's query interface
    print("\n=== Looking for clear_error service ===")
    try:
        qi = bot._query_interface
        for attr in dir(qi):
            if not attr.startswith("_"):
                print(f"  qi.{attr}")
    except Exception as ex:
        print(f"  No _query_interface: {ex}")

    # Try direct service call for clear_error
    print("\n=== Trying clear_error via query interface ===")
    try:
        result = bot._query_interface.query("clear_error", {})
        print(f"  Result: {result}")
        time.sleep(2)
    except Exception as ex:
        print(f"  Failed: {ex}")

    # Check again after clear
    print("\n=== State after clear attempt ===")
    state = arm._get_state()
    if "error" in state:
        print(f"  errors: {state['error']}")

    # Try sending command again
    print("\n=== Motor test after clear ===")
    arm.set_modes(["position"] * 7)
    original_limits = arm.joint_pos_limit.copy()
    pos = arm.get_joint_pos()
    arm._joint_pos_limit[2] = [-3.2, 3.2]

    target = pos.copy()
    target[2] = 3.021  # just inside original limit
    arm.set_joint_pos(target)
    time.sleep(2)

    actual = arm.get_joint_pos()[2]
    curr = arm.get_joint_current()[2]
    print(f"  cmd=3.021, actual={actual:.4f}, current={curr:.4f}")

    arm._joint_pos_limit[:] = original_limits
    print()
