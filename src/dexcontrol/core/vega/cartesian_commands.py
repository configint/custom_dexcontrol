"""Cartesian command conversion helpers for the Vega controller.

``target_cartesian_delta`` is a physical pose error expressed in metres and
radians.  It must pass through unchanged while it is inside the configured
per-step safety limits, and be norm-clipped only when it exceeds them.

``cartesian_velocity`` keeps its legacy normalized ``[-1, 1]`` semantics and
is therefore scaled to the same per-step limits on every command.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R


def _validate_limit(name: str, value: float) -> float:
    limit = float(value)
    if not np.isfinite(limit) or limit < 0.0:
        raise ValueError(f"{name} must be a finite non-negative value, got {value}")
    return limit


def _clip_vector_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    """Return ``vector`` unchanged below ``max_norm``, otherwise norm-clip it."""
    norm = float(np.linalg.norm(vector))
    if norm == 0.0 or norm <= max_norm:
        return vector
    return vector * (max_norm / norm)


def clip_physical_cartesian_delta(
    command: np.ndarray,
    max_linear_delta: float,
    max_rotation_delta: float,
) -> np.ndarray:
    """Clip a physical Cartesian pose delta in metres/radians.

    Linear and rotational 3-vectors are clipped independently so their
    directions are preserved.  Any trailing values, such as a gripper command,
    are copied without modification.
    """
    linear_limit = _validate_limit("max_linear_delta", max_linear_delta)
    rotation_limit = _validate_limit("max_rotation_delta", max_rotation_delta)

    converted = np.asarray(command, dtype=np.float64).copy()
    if converted.ndim != 1 or converted.shape[0] < 6:
        raise ValueError(
            "Cartesian command must be a one-dimensional array with at least 6 values"
        )

    converted[:3] = _clip_vector_norm(converted[:3], linear_limit)
    converted[3:6] = _clip_vector_norm(converted[3:6], rotation_limit)
    return converted


def normalized_cartesian_velocity_to_delta(
    command: np.ndarray,
    max_linear_delta: float,
    max_rotation_delta: float,
) -> np.ndarray:
    """Convert a legacy normalized Cartesian velocity to a per-step delta."""
    linear_limit = _validate_limit("max_linear_delta", max_linear_delta)
    rotation_limit = _validate_limit("max_rotation_delta", max_rotation_delta)

    converted = np.asarray(command, dtype=np.float64).copy()
    if converted.ndim != 1 or converted.shape[0] < 6:
        raise ValueError(
            "Cartesian command must be a one-dimensional array with at least 6 values"
        )

    linear = _clip_vector_norm(converted[:3], 1.0)
    rotation = _clip_vector_norm(converted[3:6], 1.0)
    converted[:3] = linear * linear_limit
    converted[3:6] = rotation * rotation_limit
    return converted


def absolute_cartesian_target_to_delta(
    target_pose: np.ndarray,
    current_pose: np.ndarray,
    max_linear_delta: float,
    max_rotation_delta: float,
) -> np.ndarray:
    """Convert an absolute Cartesian target into a bounded world-frame delta.

    Both poses use ``[x, y, z, roll, pitch, yaw]`` in metres/radians.  The
    returned rotation is an XYZ Euler delta whose left-multiplication onto the
    current orientation reaches the target orientation.  Rotation clipping is
    performed on the relative rotation vector, so ``max_rotation_delta`` is a
    physical angle bound rather than an Euler-component approximation.
    """
    linear_limit = _validate_limit("max_linear_delta", max_linear_delta)
    rotation_limit = _validate_limit("max_rotation_delta", max_rotation_delta)

    target = np.asarray(target_pose, dtype=np.float64)
    current = np.asarray(current_pose, dtype=np.float64)
    if target.shape != (6,) or current.shape != (6,):
        raise ValueError(
            "target_pose and current_pose must each have shape (6,)"
        )

    delta_xyz = _clip_vector_norm(target[:3] - current[:3], linear_limit)

    target_rotation = R.from_euler("xyz", target[3:6])
    current_rotation = R.from_euler("xyz", current[3:6])
    relative_rotation = target_rotation * current_rotation.inv()
    delta_rotvec = _clip_vector_norm(
        relative_rotation.as_rotvec(),
        rotation_limit,
    )
    delta_rpy = R.from_rotvec(delta_rotvec).as_euler("xyz")

    return np.concatenate([delta_xyz, delta_rpy]).astype(np.float64)
