"""Reboot arm component and immediately command joint back within limits.

Strategy: Use reboot_component("arm") to reset just the arm firmware,
then immediately send position commands before the position limit
error is re-triggered.

This is faster than a full robot power cycle because the Robot object
is already connected and ready to send commands instantly after reboot.
"""

import time
import numpy as np
from loguru import logger
from dexcontrol.robot import Robot

side = "left"
joint_idx = 2  # L_arm_j3
margin = 0.05

logger.warning("=== ARM REBOOT RECOVERY ===")
logger.warning("This will reboot the arm controller and immediately try to move the joint.")
logger.warning("Be ready to press e-stop!")
if input("Continue? [y/N]: ").strip().lower() != "y":
    exit()

with Robot() as bot:
    arm = bot.left_arm if side == "left" else bot.right_arm
    part = "left_arm" if side == "left" else "right_arm"

    pos = arm.get_joint_pos()
    limits = arm.joint_pos_limit
    names = arm.joint_name
    cur_pos = pos[joint_idx]
    lo, hi = limits[joint_idx]

    logger.info(f"{names[joint_idx]}: pos={cur_pos:.4f}, limits=[{lo:.4f}, {hi:.4f}]")

    if lo <= cur_pos <= hi:
        logger.info("Already within limits. Nothing to do.")
        exit()

    # Expand software limits
    arm._joint_pos_limit[joint_idx] = [min(lo, cur_pos - 0.2), max(hi, cur_pos + 0.2)]

    # Prepare target
    target = hi - margin if cur_pos > hi else lo + margin
    pos_cmd = pos.copy()
    pos_cmd[joint_idx] = target

    logger.info(f"Target: {target:.4f}")
    logger.info(f"Will reboot arm and immediately send commands...")

    # REBOOT the arm component
    logger.warning("Rebooting arm...")
    try:
        bot.reboot_component("arm")
    except Exception as e:
        logger.error(f"Reboot failed: {e}")
        arm._joint_pos_limit[joint_idx] = [lo, hi]
        exit()

    # Immediately start hammering commands
    # The arm needs time to come back online, so we do a tight loop
    logger.info("Sending position commands immediately...")
    start_time = time.time()

    for i in range(500):  # 10 seconds of attempts
        try:
            # Try clearing error (may fail if arm not ready yet)
            if i % 50 == 0:
                try:
                    bot.clear_error(part)
                except Exception:
                    pass

            # Try setting position mode
            if i % 50 == 0:
                try:
                    arm.set_modes(["position"] * 7)
                except Exception:
                    pass

            # Send position command
            arm._publish_control({"pos": pos_cmd})
            time.sleep(0.02)

            if i % 25 == 0:
                try:
                    actual = arm.get_joint_pos()[joint_idx]
                    curr = arm.get_joint_current()[joint_idx]
                    elapsed = time.time() - start_time
                    err = arm._get_state().get("error", {})
                    logger.info(
                        f"  t={elapsed:.1f}s: pos={actual:.4f}, current={curr:.4f}, "
                        f"error={bool(err)}, moved={actual-cur_pos:.4f}"
                    )

                    if lo <= actual <= hi:
                        logger.info("Joint is within limits!")
                        break

                    if abs(actual - cur_pos) > 0.005:
                        logger.info("Movement detected! Continuing...")

                except Exception as e:
                    elapsed = time.time() - start_time
                    logger.info(f"  t={elapsed:.1f}s: arm not ready yet ({e})")

        except Exception:
            time.sleep(0.02)

    # Restore limits
    arm._joint_pos_limit[joint_idx] = [lo, hi]

    try:
        final = arm.get_joint_pos()[joint_idx]
        if lo <= final <= hi:
            logger.info(f"RECOVERED! {names[joint_idx]}: {final:.4f}")
        else:
            logger.warning(f"Still over limit: {final:.4f}")
            logger.info("Even after arm reboot, the firmware re-locks too fast.")
    except Exception:
        logger.warning("Could not read final position (arm may still be rebooting)")
