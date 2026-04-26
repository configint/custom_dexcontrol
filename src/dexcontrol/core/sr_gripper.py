# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""SR gripper adapter for dexcontrol.

Wraps `sr_gripper_controller.SrGripper` to expose the same interface as
DexGripper/Hand so it can be used as a drop-in replacement in VegaRobot.

Requirements:
    pip install -e sr_gripper_controller/
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
from loguru import logger

# Ensure the submodule takes precedence over any namespace package with the
# same name that may shadow it.
_SUBMODULE = Path(__file__).resolve().parents[3] / "sr_gripper_controller"
if _SUBMODULE.exists() and str(_SUBMODULE) not in sys.path:
    sys.path.insert(0, str(_SUBMODULE))

try:
    from sr_gripper_controller import SrGripper
except ImportError as e:
    raise ImportError(
        "sr_gripper_controller is not installed. "
        "Run: pip install -e sr_gripper_controller/"
    ) from e

# SR gripper stroke in metres (full travel: 0 m open .. 0.11 m closed).
_STROKE_M = 0.11

# Predefined positions in metres. Mirrors the dexcontrol Robotiq wrapper's
# mapping so VegaRobot pairs `open`/`close` symbolically the same way.
_POSE_POOL = {
    "open": np.array([_STROKE_M], dtype=np.float64),
    "close": np.array([0.0], dtype=np.float64),
}


class SrGripperAdapter:
    """SR gripper controller compatible with the VegaRobot hand interface.

    Joint space is a single scalar in metres [0, 0.11]. The raw `goto(pos)`
    semantics are exposed directly (matching the dexcontrol Robotiq wrapper's
    behaviour); VegaRobot drives the gripper through `get_predefined_pose`
    keys, so the symbolic open/close mapping is what matters here.
    """

    def __init__(
        self,
        comport: str = "eth0",
        init_timeout: float = 15.0,
    ) -> None:
        """Connect to the gripper and run full initialisation.

        Args:
            comport: EtherCAT network interface the gripper is on.
            init_timeout: Seconds to wait for the gripper to become ready.
        """
        logger.info("Connecting to SR gripper on %s …", comport)
        self._gripper = SrGripper(comport=comport, stroke=_STROKE_M)
        self._gripper.full_init(timeout=init_timeout)
        logger.info("SR gripper ready.")

    def get_joint_pos(self) -> np.ndarray:
        """Return current gripper position as a 1-element array in metres."""
        self._gripper.getStatus()
        return np.array([self._gripper.get_pos()], dtype=np.float64)

    def set_joint_pos(
        self,
        joint_pos,
        wait_time: float = 0.0,
        **_kwargs,
    ) -> None:
        """Command gripper to a position in metres.

        Args:
            joint_pos: Target position. Scalar, list, or 1-element array in
                metres, clipped to [0, stroke].
            wait_time: Seconds to sleep after sending the command.
        """
        pos_m = float(np.asarray(joint_pos, dtype=np.float64).flat[0])
        pos_m = float(np.clip(pos_m, 0.0, _STROKE_M))
        self._gripper.goto(pos=pos_m, vel=0.05, force=95)
        self._gripper.sendCommand()
        if wait_time > 0.0:
            time.sleep(wait_time)

    def open_hand(self, wait_time: float = 0.0, **_kwargs) -> None:
        """Open the gripper fully."""
        self._gripper.goto(pos=_STROKE_M, vel=0.05, force=95)
        self._gripper.sendCommand()
        if wait_time > 0.0:
            time.sleep(wait_time)

    def close_hand(self, wait_time: float = 0.0, **_kwargs) -> None:
        """Close the gripper fully."""
        self._gripper.goto(pos=0.0, vel=0.05, force=95)
        self._gripper.sendCommand()
        if wait_time > 0.0:
            time.sleep(wait_time)

    def get_predefined_pose(self, name: str) -> np.ndarray:
        """Return a predefined pose by name ('open' or 'close').

        Args:
            name: Pose name.

        Returns:
            1-element numpy array with position in metres.

        Raises:
            KeyError: If name is not a known predefined pose.
        """
        if name not in _POSE_POOL:
            raise KeyError(f"Unknown predefined pose '{name}'. Available: {list(_POSE_POOL)}")
        return _POSE_POOL[name].copy()

    def shutdown(self) -> None:
        """Disconnect from the gripper."""
        try:
            self._gripper.shutdown()
        except Exception:
            pass
