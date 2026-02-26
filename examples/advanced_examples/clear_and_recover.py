"""Clear motor error and immediately nudge over-limit joint back within limits.

Key insight: after clear_error, the firmware may re-trigger the error
immediately because the joint is still over limit. We must:
1. Expand software limits FIRST
2. Set position mode FIRST
3. Clear the error
4. IMMEDIATELY send a position command toward the limit before the error re-triggers
"""

import time
import numpy as np
from loguru import logger
from dexcontrol.robot import Robot

side = "left"
joint = 2
margin = 0.05  # target this far inside limit (rad)

logger.warning("WARNING: This will command the motor to move.")
logger.warning("Be ready to press e-stop if needed!")
if input("Continue? [y/N]: ").strip().lower() != "y":
    exit()

with Robot() as bot:
    arm = bot.left_arm if side == "left" else bot.right_arm
    part = "left_arm" if side == "left" else "right_arm"

    pos = arm.get_joint_pos()
    limits = arm.joint_pos_limit
    names = arm.joint_name
    cur_pos = pos[joint]

    logger.info(f"{names[joint]}: pos={cur_pos:.4f}, limits=[{limits[joint][0]:.4f}, {limits[joint][1]:.4f}]")

    state = arm._get_state()
    logger.info(f"Error state: {state.get('error', 'none')}")

    # Step 1: Expand software limits BEFORE anything else
    original_limits = arm.joint_pos_limit.copy()
    arm._joint_pos_limit[joint] = [min(limits[joint][0], cur_pos - 0.2),
                                    max(limits[joint][1], cur_pos + 0.2)]
    logger.info(f"Expanded software limits for {names[joint]}")

    # Step 2: Set position mode
    arm.set_modes(["position"] * 7)
    logger.info("Motors set to position mode")

    # Step 3: Prepare the target command
    target = limits[joint][1] - margin if cur_pos > limits[joint][1] else limits[joint][0] + margin
    cmd = arm.get_joint_pos().copy()
    cmd[joint] = target

    # Step 4: Clear error and IMMEDIATELY send command (no sleep!)
    logger.warning("Clearing error and immediately sending position command...")
    bot.clear_error(part)
    arm.set_joint_pos(cmd)  # send immediately!

    # Rapidly keep sending commands for a few seconds
    for i in range(100):
        arm.set_joint_pos(cmd)
        time.sleep(0.02)
        if i % 25 == 0:
            actual = arm.get_joint_pos()[joint]
            curr = arm.get_joint_current()[joint]
            err = arm._get_state().get("error", {})
            logger.info(f"  t={i*0.02:.1f}s: actual={actual:.4f}, current={curr:.4f}, error={err}")

    time.sleep(1)
    final = arm.get_joint_pos()[joint]
    lo, hi = limits[joint]
    arm._joint_pos_limit[:] = original_limits
    logger.info("Software limits restored")

    if lo <= final <= hi:
        logger.info(f"{names[joint]} recovered! pos={final:.4f}")
    else:
        logger.warning(f"{names[joint]} still over limit: {final:.4f}")
        logger.info("The firmware may re-lock too fast. Possible options:")
        logger.info("  1. Update firmware to a version that supports brake release")
        logger.info("  2. Contact Dexmate support for manual recovery procedure")
