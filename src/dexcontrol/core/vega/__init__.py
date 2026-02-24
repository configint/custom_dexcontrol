"""Vega robot interfaces."""

from core.vega.robot import (
    CommunicationFailedError,
    IKFailedError,
    JointLimitExceededError,
    VegaRobot,
)

__all__ = [
    "VegaRobot",
    "JointLimitExceededError",
    "IKFailedError",
    "CommunicationFailedError",
]
